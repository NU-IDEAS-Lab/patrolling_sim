import networkx as nx
import rclpy
import torch
import numpy as np
from gymnasium import spaces
from gymnasium.spaces.utils import flatten, flatten_space
from onpolicy.algorithms.r_mappo.algorithm.r_actor_critic import R_Actor

from onpolicy.scripts.render.render_patrolling import get_config, parse_args

import random
import os
import yaml

from std_msgs.msg import Float32MultiArray

from patrol_algorithms_cdc2023.BasePatrolAgent import BasePatrolAgent

from patrolling_zoo.env.patrolling_zoo import PatrolAgent as PzPatrolAgent
from patrolling_zoo.env.patrolling_zoo import parallel_env as pz_parallel_env
from onpolicy.envs.patrolling.Patrolling_Env import PatrollingEnv

from ament_index_python.packages import get_package_share_directory


def _t2n(x):
    return x.detach().cpu().numpy()



class PzAgent(BasePatrolAgent):
    def __init__(self):
        self.pzReady = False
        super().__init__()

        # Set the model directory using a ROS 2 parameter.
        # self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/run-20231006_202648-p5johle2/files")
        # self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/policies/3attritionNoCommsNoSkipAsyncAdjacency/wandb/run-20231008_162207-l8n539te/files") #TEST2
        # self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/policies/6attritionYesCommsNoSkipAsyncAdjacency/wandb/run-20231008_162207-ftmpttrx/files") #TEST3
        # self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/policies/6SpAttritionYesCommsNoSkipAsyncAdjacency/wandb/run-20231009_100332-0w7n9mb0/files") #TEST4
        # self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/patrolling_zoo/onpolicy/scripts/results/Patrolling/cumberland/rmappo/1attritionYesComms01SkipAsyncBitmap2/wandb/run-20231007_220950-wlz4r9v5/files")
        self.declare_parameter("model_dir", "/home/anthony/papers/aamas2024/policies/6attritionYesCommsNoSkipAsyncBitmap2/wandb/run-20231008_155309-f4gvl2ju/files")
        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value

        # self.get_logger().info(f"Here is the initialize of the PZ agent")

        # Set the allocation.
        # self.currentNodeIdx = self.nodes.index(self.agentOrigins[self.id])

        self.model_dir = model_dir


        #Load the default arguments
        self.parser = get_config()
        self.all_args = parse_args([], self.parser)

        #Load new arguments to Namespace object
        config_file = os.path.join(model_dir, "config.yaml")
        self.args = yaml.load(open(config_file), Loader=yaml.FullLoader)


        # Convert the arguments to a Namespace object
        for a in self.args:
            if type(self.args[a]) == dict and "value" in self.args[a]:
                self.args[a] = self.args[a]["value"]
        for a in self.args:
            setattr(self.all_args, a, self.args[a])

        
        # Check args.
        if self.all_args.num_agents != self.agent_count and self.all_args.observe_method != "bitmap2":
            # We can easily avoid this problem if we use the bitmap2 observation mode.
            raise ValueError(f"Number of agents in the environment ({self.agent_count}) does not match the number of agents in the model ({self.all_args.num_agents})!")


        # Set required specific argument
        self.all_args.use_wandb = False
        self.all_args.model_dir = model_dir
        self.all_args.cuda_idx = 0
        self.all_args.graph_file = os.path.join(get_package_share_directory("patrolling_sim"), "models/maps/cumberland/cumberland_full_res.graph")

        # self.all_args.communication_model = "none"
        # self.all_args.communication_probability = 0.0
        self.all_args.observation_radius = np.inf

        # Set up environment
        self.env = PatrollingEnv(self.all_args)

        # Set initial nodes in environment.
        for agent in self.env.env.possible_agents:
            agent.lastNode = self.agentOrigins[agent.id]
        
        # Create a local agents dictionary.
        self.agentsDict = {agent.id: agent for agent in self.env.env.possible_agents}

        self.obs_space_orig = self.env.env.observation_spaces[self.env.env.possible_agents[self.id]]
        self.obs_space = self.env.observation_space[0]
        # self.get_logger().info(f"the type of obs_space is {self.obs_space.__class__.__name__}")
        # self.get_logger().info("here finished the buildStateSpace")
        # self.get_logger().info(f"here is the size of obs space {len(self.obs_space.shape)}")

        self.action_space = self.env.action_space[0]
        # self.get_logger().info("here finished the ActionSpace")

        self.recurrent_N = self.all_args.recurrent_N
        self.hidden_size = self.all_args.hidden_size
        self.actor = R_Actor(self.all_args, self.obs_space, self.action_space)
        # for name, layer in self.actor.named_children():
        #     self.get_logger().info(f"here is name {name} and layer {layer}")

        checkpoint = torch.load(os.path.join(model_dir, f"actor_agent{self.id}.pt"), map_location=torch.device("cpu"))

        # self.get_logger().info(f"here is the type of checkpoint {type(checkpoint)}")
        self.actor.load_state_dict(checkpoint)
        self.rnn_states = np.zeros((1, 1, self.recurrent_N, self.hidden_size), dtype=np.float32)
        self.masks = np.ones((1, 1, 1), dtype=np.float32)

        self.path = []

        # Subscribe to the /idleness topic.
        self.subIdleness = self.create_subscription(
            Float32MultiArray,
            "/idleness",
            self.onReceiveIdleness,
            100
        )

        self.get_logger().info(f"PZ agent {self.id} initialization complete.")
        self.pzReady = True


    def onReceiveIdleness(self, msg):
        ''' Called when a new idleness vector is received. '''

        timeNow = self.get_clock().now().to_msg().sec

        # Update the idleness.
        for v in range(self.graph.graph.number_of_nodes()):
            idle = timeNow - msg.data[v]
            self.env.env.pg.setNodeVisitTime(v, idle)

    def onReceiveTelemetry(self, msg):
        super().onReceiveTelemetry(msg)

        if self.pzReady and msg.sender in self.agentsDict:
            self.agentsDict[msg.sender].position = (msg.odom.pose.pose.position.x / self.graph.resolution,
                                            msg.odom.pose.pose.position.y / self.graph.resolution)


    def onAgentAttrition(self, agentId):
        super().onAgentAttrition(agentId)

        self.get_logger().info(f"PZ agent {self.id} removing dead agent {agentId} from environment")
        self.env.env.agents.remove(self.agentsDict[agentId])


    def onNavigationGoalSuccess(self):

        # Set visit time.
        secsNow = self.get_clock().now().to_msg().sec
        self.env.env.pg.setNodeVisitTime(self.goalNode, secsNow)

        self.agentsDict[self.id].edge = None
        self.agentsDict[self.id].lastNode = self.goalNode
        super().onNavigationGoalSuccess()
    
    def getNextNode(self):
        ''' Returns the next node to visit. '''

        # self.get_logger().info("here is the start of PZ Agent Action")

        # Get new goal node.
        secsNow = self.get_clock().now().to_msg().sec
        if len(self.path) == 0:
            self.env.env.step_count = secsNow
            # very hacky...

            self.get_logger().info(f"Agent {self.id} choosing new goal. Avg idleness: {np.mean(self.env.env.pg.getAverageIdlenessTime(self.env.env.step_count))}")

            # observations = {agent: self.env.env.observe(agent, allow_done_agents=False) for agent in self.env.env.possible_agents}
            # if self.id == 0:
            #     self.get_logger().warn(f"Raw observation: {observations}")
            # observations = self.env._obs_wrapper(observations)
            # obs = observations[self.id]
            obs = self.env.env.observe(self.env.env.possible_agents[self.id], allow_done_agents=False)

            # if self.id == 0:
            #     self.get_logger().warn(f"Wrapped observation: {obs}")

            # from matplotlib import pyplot as plt
            # # bitmap = self.env.env.observe(self.env.env.agents[0], radius=200)
            # bitmap = obs
            # graphLayer = bitmap[:,:,2]
            # graphNodes = graphLayer[np.where(graphLayer >= 0)]
            # self.get_logger().info(f"Sum of node layer: {np.sum(graphNodes)}")
            # self.get_logger().info(f"Expected sum of node layer: {sum(list(range(len(self.env.env.pg.graph.nodes))))}")

            # agentLayer = bitmap[:,:,0]
            # agents = agentLayer[np.where(agentLayer >= 0)]

            # if bitmap[0,0,0] != self.id:
            #     self.get_logger().error(f"Agent ID {bitmap[0,0,0]} does not match expected ID {self.id}!")

            # self.get_logger().info(f"Sum of agent layer: {np.sum(agents)}")
            # self.get_logger().info(f"Expected sum of agent layer: {sum(list(range(self.agent_count)))}")
            # if np.sum(agents) != sum(list(range(self.agent_count))) + self.id:
            #     self.get_logger().error(f"Agent layer does not match expected sum!")


            # bitmap = self.env.env._minMaxNormalize(bitmap, a=0, b=255)
            # bitmap[np.where(graphLayer >= 0)] = 255
            # bitmap[np.where(agentLayer >= 0)] = 0
            # plt.imshow(bitmap, aspect="equal")
            # # save image using pyplot
            # plt.savefig(f"obs_{self.id}_{self.env.env.step_count}.png")


            # self.get_logger().info("here is the start of PZ Agent Action process")
            # self.obs_space_new = spaces.Dict(self.t)
            # self.get_logger().info("here is the start of PZ Agent Action process")
            if self.env.flatten_observations:
                # self.get_logger().warn(f"{type(self.obs_space)}, {type(obs)}, OBS SPACE: {self.obs_space}, OBS: {obs}")
                obs = flatten(self.obs_space_orig, obs)
                obs = obs.reshape((1,-1))
            else:
                obs = np.expand_dims(obs, axis=0)
            # self.get_logger().info(f"here is the shape of obs of input rnn {obs.shape}, {self.rnn_states[:,0].shape}, {self.masks[:,0].shape}")
            action, action_log_probs, rnn_state = self.actor(obs, self.rnn_states[:,0], self.masks[:,0], deterministic=True)
            action = action.item()
            # self.get_logger().info("here is the mid process")
            self.rnn_states[0,0] = np.array(np.split(_t2n(rnn_state), 1))
            # self.get_logger().info(f"the action of PZ Agent Action {action}")
            
            # Get path to the goal node.
            self.pzGoalNode = action
            self.path = self.env.env._getPathToNode(self.agentsDict[self.id], self.pzGoalNode)
        
        nextNode = self.path.pop(0)
        self.agentsDict[self.id].edge = (self.agentsDict[self.id].lastNode, nextNode)
        # nextNode = self.pzGoalNode
        self.get_logger().info(f"PZ Agent {self.id} moving to node {nextNode} along edge {self.agentsDict[self.id].edge} with idleness {self.env.env.pg.getNodeIdlenessTime(nextNode, secsNow):.2f}. Remaining path: {self.path}")
        return nextNode
        


def main(args=None):
    # Initialize ROS2.
    rclpy.init(args=args)

    # Create the node and execute.
    agent = PzAgent()
    rclpy.spin(agent)

    # Destroy the node explicitly.
    agent.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()