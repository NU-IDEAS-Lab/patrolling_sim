import torch
import numpy as np
from gymnasium.spaces.utils import flatten
from onpolicy.algorithms.r_mappo.algorithm.r_actor_critic import R_Actor

from onpolicy.scripts.render.render_patrolling import get_config, parse_args

import os
import yaml

from patrolling_sim_interfaces.srv import QueryPolicy

from onpolicy.envs.patrolling.Patrolling_Env import PatrollingEnv

def _t2n(x):
    return x.detach().cpu().numpy()

class PzEnvironment:
    def __init__(self, node):
        self.pzReady = False
        self.node = node
        super().__init__()

        # Set the model directory using a ROS 2 parameter.
        self.node.declare_parameter("agent_policy_dir", "/home/anthony/papers/aamas2024/policies/6attritionYesCommsNoSkipAsyncBitmap2/wandb/run-20231008_155309-f4gvl2ju/files")
        self.node.declare_parameter("lost_message_rate", 0.0)
        self.model_dir = self.node.get_parameter("agent_policy_dir").get_parameter_value().string_value
        self.lost_message_rate = self.node.get_parameter("lost_message_rate").get_parameter_value().double_value
        self.graphFilePath = self.node.graphFilePath

        #Load the default arguments
        self.parser = get_config()
        self.all_args = parse_args([], self.parser)

        #Load new arguments to Namespace object
        config_file = os.path.join(self.model_dir, "config.yaml")
        self.args = yaml.load(open(config_file), Loader=yaml.FullLoader)

        # Convert the arguments to a Namespace object
        for a in self.args:
            if type(self.args[a]) == dict and "value" in self.args[a]:
                self.args[a] = self.args[a]["value"]
        for a in self.args:
            setattr(self.all_args, a, self.args[a])

        # Set required specific arguments.
        self.all_args.num_agents = self.node.agent_count
        self.all_args.use_wandb = False
        self.all_args.model_dir = self.model_dir
        self.all_args.cuda_idx = 0
        self.all_args.graph_random = False
        self.all_args.graph_file = self.graphFilePath
        self.all_args.communication_model = "bernoulli"
        self.all_args.communication_probability = 1.0 - self.lost_message_rate
        self.node.get_logger().info(f"PZ environment is using graph file {self.all_args.graph_file}")

        # Scale the observation radius.
        self.all_args.observation_radius /= self.node.graph.resolution

        self.node.get_logger().info(f"PZ environment is using observation radius {self.all_args.observation_radius} and communication probability {self.all_args.communication_probability}")

        # Set up environment
        self.env = PatrollingEnv(self.all_args)

        # Set initial nodes in environment.
        for agent in self.env.env.possible_agents:
            agent.lastNode = self.node.agentOrigins[agent.id]
            agent.edge = None
            agent.position = self.node.agentPositions[agent.id]
        
        # Create a local agents dictionary.
        self.agentsDict = {agent.id: agent for agent in self.env.env.possible_agents}

        self.obs_space_orig = self.env.env.observation_spaces[self.env.env.possible_agents[0]]
        self.obs_space = self.env.observation_space[0]

        self.action_space = self.env.action_space[0]

        self.recurrent_N = self.all_args.recurrent_N
        self.hidden_size = self.all_args.hidden_size
        self.actor = R_Actor(self.all_args, self.obs_space, self.action_space)

        checkpoint = torch.load(os.path.join(self.model_dir, f"actor.pt"), map_location=torch.device("cpu"))

        self.actor.load_state_dict(checkpoint)
        self.rnn_states = np.zeros((1, 1, self.recurrent_N, self.hidden_size), dtype=np.float32)
        self.masks = np.ones((1, 1, 1), dtype=np.float32)

        # Create a ROS 2 service for agents to query the policy.
        # We do this as a convenience. In reality, agents can load the model and query independently.
        self.node.create_service(QueryPolicy, f"/pz_query_policy", self.onQueryPolicy)

        self.node.get_logger().info(f"PzEnvironment initialization complete.")
        self.pzReady = True


    def onQueryPolicy(self, request, response):
        ''' Callback for the get_next_node service. '''

        # First we mark the agent as having completed the prior goal.
        # This avoids a potential race condition.
        self.onNavigationGoalSuccess(request.agent_id)

        response.node_id = self.getNextNode(request.agent_id)
        return response


    def onReceiveTelemetry(self, msg):
        self.agentsDict[msg.sender].position = (msg.odom.pose.pose.position.x / self.node.graph.resolution,
                                        msg.odom.pose.pose.position.y / self.node.graph.resolution)


    def onAgentAttrition(self, agentId):
        self.node.get_logger().info(f"PzEnvironment removing dead agent {agentId} from environment")
        self.env.env.agents.remove(self.agentsDict[agentId])


    def onNavigationGoalSuccess(self, agentId, goalNode = None):
        ''' Called when an agent has reached its goal. '''

        if goalNode is None:
            if self.agentsDict[agentId].edge is None:
                self.node.get_logger().warn(f"PzEnvironment ignoring goal success for agent {agentId} as it has no edge and no goal was specified.")
                return
            goalNode = self.agentsDict[agentId].edge[1]

        # Set visit time.
        timeElapsed = self.node.getTimeElapsed().nanoseconds / 1.0e9
        self.env.env.pg.setNodeVisitTime(goalNode, timeElapsed)

        if goalNode != self.agentsDict[agentId].lastNode:
            self.agentsDict[agentId].edge = None
            self.agentsDict[agentId].lastNode = goalNode
    
    def getNextNode(self, agentId):
        ''' Returns the next node to visit for agent. '''

        # Get new goal node.
        # very hacky...
        timeElapsed = self.node.getTimeElapsed().nanoseconds / 1.0e9
        self.env.env.step_count = timeElapsed

        self.node.get_logger().info(f"Agent {agentId} choosing new goal. Avg idleness: {np.mean(self.env.env.pg.getAverageIdlenessTime(self.env.env.step_count))}")


        # Print out the location of every active agent.
        msg = ""
        for agent in self.env.env.agents:
            msg += f"Agent {agent.id} at node {agent.lastNode}, edge {agent.edge}, position {agent.position}\n"
        self.node.get_logger().info(msg)

        obs = self.env.env.observe(self.agentsDict[agentId], allow_done_agents=False)

        if self.env.flatten_observations:
            obs = flatten(self.obs_space_orig, obs)
            obs = obs.reshape((1, -1))
        else:
            obs = np.expand_dims(obs, axis=0)

        # Get available actions.
        available_actions = self.env.env._getAvailableActions(self.agentsDict[agentId])
        available_actions = np.expand_dims(available_actions, axis=0)

        action, action_log_probs, rnn_state = self.actor(obs, self.rnn_states[:,0], self.masks[:,0], available_actions=available_actions)
        action = action.item()
        self.node.get_logger().info(f"PZ Agent {agentId} chose action {action} with log prob {action_log_probs.item()}. Available actions: {available_actions}")
        self.rnn_states[0,0] = np.array(np.split(_t2n(rnn_state), 1))
        
        # Get the goal node.
        nextNode = self.env.env.getDestinationNode(self.agentsDict[agentId], action)
        
        self.agentsDict[agentId].edge = (self.agentsDict[agentId].lastNode, nextNode)
        self.node.get_logger().info(f"PZ Agent {agentId} moving to node {nextNode} along edge {self.agentsDict[agentId].edge} with idleness {self.env.env.pg.getNodeIdlenessTime(nextNode, timeElapsed):.2f}.")
        return nextNode
