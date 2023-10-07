import networkx as nx
import rclpy
import torch
import numpy as np
from gymnasium import spaces
from gymnasium.spaces.utils import flatten, flatten_space
from .r_actor_critic import R_Actor

from onpolicy.scripts.render.render_patrolling import get_config, parse_args

import os
import yaml

from std_msgs.msg import Int32MultiArray

from patrol_algorithms_cdc2023.BasePatrolAgent import BasePatrolAgent




def _t2n(x):
    return x.detach().cpu().numpy()



class PzAgent(BasePatrolAgent):
    def __init__(self, model_dir = "/home/xinliangli/Downloads/configuration"):
        super().__init__()

        self.get_logger().info(f"PZ agent {self.id} has origin {self.agentOrigins[self.id]}.")

        # Set the allocation.
        self.currentNodeIdx = self.nodes.index(self.agentOrigins[self.id])


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


        # Set required specific argument
        self.all_args.use_wandb = False
        self.all_args.model_dir = model_dir


        # set the state of this agent in this environment
        self.adjacency_obs = {}
        self.adjacency_obs["agent_id"] = self.id

        self.obs_space = flatten_space(self._buildStateSpace())
        self.action_space = spaces.Discrete(len(self.graph))
        self.recurrent_N = self.all_args.recurrent_N
        self.hidden_size = self.all_args.hidden_size
        self.actor = R_Actor(self.all_args, self.obs_space, self.action_space)
        checkpoint = torch.load("actor_agent0.pt")
        self.actor.load_state_dict(checkpoint)
        self.rnn_states = np.zeros((1, 1, self.recurrent_N, self.hidden_size), dtype=np.float32)
        self.masks = np.ones((1, 1, 1), dtype=np.float32)

        # Subscribe to the /idleness topic.
        self.subIdleness = self.create_subscription(
            Int32MultiArray,
            "/idleness",
            self.onReceiveIdleness,
            100
        )


        self.get_logger().info(f"Patrol order: {self.nodes}")


    def onReceiveIdleness(self, msg):
        ''' Called when a new idleness vector is received. '''

        print(f"Received idleness information: {msg.data}")
        # Xinliang, this is where you get the idleness information!


    def set_state(self):
        self.adjacency_obs["agent_id"] = self.id 
        self.adjacency_obs["vertex_state"] = {v: 0.0 for v in range(self.graph.number_of_nodes())}
        self.adjacency_obs["adjacency"] = -1.0 * np.ones((self.pg.graph.number_of_nodes(), self.pg.graph.number_of_nodes()), dtype=np.float32)
        graphPos = {}
        for i in range(self.agent_count):
            graphPos[i] = -1.0 * np.ones(3, dtype=np.float32)
        self.adjacency_obs["agent_graph_position"] = graphPos
    
    def getNextNode(self):
        ''' Returns the next node to visit. '''

        obs = self.set_state()
        obs = flatten(self.obs_space, obs)
        obs = np.array(obs)
        action, rnn_state = self.actor(obs, self.rnn_states[:,0], self.masks[:,0], deterministic=True)
        self.rnn_states[0,0] = np.array(np.split(_t2n(rnn_state), 1))
        print(action)
        return action
    
    def onAgentAttrition(self, agent):
        ''' Called when an agent is lost. '''

        super().onAgentAttrition(agent)
        if agent == self.id:
            return

        # Update the Voronoi partitions.
        del self.voronoiOrigins[agent]
        cell = self.getNodeAllocation(self.voronoiOrigins, self.agentOrigins)
        old = self.nodes
        self.nodes = self.getNodeOrder(cell)

        if old != self.nodes:
            self.get_logger().info(f"Updated allocation!\nOld patrol route: {old}\nNew patrol route: {self.nodes}")


    def _buildStateSpace(self):
        state_space = {}
        state_space["agent_id"] = spaces.Box(
            low = -1,
            high = len(self.agent_count),
            dytype = np.int32
        )

        state_space["vertex_state"] = spaces.Dict({
            v:spaces.Box(
                low = -1.0,
                high = np.inf,
            ) for v in range(self.graph.number_of_nodes())
        })

        state_space["adjacency"] = spaces.Box(
            low = -1.0,
            high = 1.0,
            shape = (self.graph.number_of_nodes(), self.graph.number_of_nodes),
            dtype = np.float32,
        )

        state_space["agent_graph_position"] = spaces.Dict({
                a: spaces.Box(
                    low = np.array([-1.0, -1.0, -1.0], dtype=np.float32),
                    high = np.array([self.pg.graph.number_of_nodes(), self.pg.graph.number_of_nodes(), 1.0], dtype=np.float32),
                ) for a in self.agent_count
            }) # type: ignore



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