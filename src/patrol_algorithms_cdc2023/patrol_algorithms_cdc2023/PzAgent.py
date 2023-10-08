import networkx as nx
import rclpy
import torch
import numpy as np
from gymnasium import spaces
from gymnasium.spaces.utils import flatten, flatten_space
from onpolicy.algorithms.r_mappo.algorithm.r_actor_critic import R_Actor

from onpolicy.scripts.render.render_patrolling import get_config, parse_args

import os
import yaml

from std_msgs.msg import Float32MultiArray

from patrol_algorithms_cdc2023.BasePatrolAgent import BasePatrolAgent

from patrolling_zoo.env.patrolling_zoo import PatrolAgent as PzPatrolAgent
from patrolling_zoo.env.patrolling_zoo import parallel_env as pz_parallel_env



def _t2n(x):
    return x.detach().cpu().numpy()



class PzAgent(BasePatrolAgent):
    def __init__(self, model_dir = "/home/anthony/papers/aamas2024/run-20231006_202648-p5johle2/files"):
        super().__init__()

        self.get_logger().info(f"Here is the initialize of the PZ agent")

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

        # Hacky stuff, make this compatible with the PZ code...
        self.pg = self.graph
        self.step_count = 0 # not even used....
        self.observe_bitmap_dims = (self.all_args.observe_bitmap_size, self.all_args.observe_bitmap_size)
        self.OBSERVATION_CHANNELS = pz_parallel_env.OBSERVATION_CHANNELS

        # Create a PzPatrolAgent for each agent in sim.
        self.agents = []
        for i in range(self.agent_count):
            self.agents.append(PzPatrolAgent(i, observationRadius=self.all_args.observation_radius))
        self.possible_agents = self.agents

        self.t= pz_parallel_env._buildStateSpace(self, "bitmap2")
        self.obs_space = flatten_space(self.t)
        self.get_logger().info(f"the type of obs_space is {self.obs_space.__class__.__name__}")
        self.get_logger().info("here finished the buildStateSpace")
        self.get_logger().info(f"here is the size of obs space {len(self.obs_space.shape)}")
        # self.obs_space = flatten_space(t)

        self.action_space = spaces.Discrete(len(self.graph.graph))
        self.get_logger().info("here finished the ActionSpace")

        self.recurrent_N = self.all_args.recurrent_N
        self.hidden_size = self.all_args.hidden_size
        self.actor = R_Actor(self.all_args, self.obs_space, self.action_space)
        # for name, layer in self.actor.named_children():
        #     self.get_logger().info(f"here is name {name} and layer {layer}")

        checkpoint = torch.load(os.path.join(model_dir, "actor_agent0.pt"), map_location=torch.device("cpu"))

        # self.get_logger().info(f"here is the type of checkpoint {type(checkpoint)}")
        self.actor.load_state_dict(checkpoint)
        self.rnn_states = np.zeros((1, 1, self.recurrent_N, self.hidden_size), dtype=np.float32)
        self.masks = np.ones((1, 1, 1), dtype=np.float32)

        # Create dictionary to store idleness information.
        self.idleness = {v: 0.0 for v in range(self.graph.graph.number_of_nodes())}

        # Subscribe to the /idleness topic.
        self.subIdleness = self.create_subscription(
            Float32MultiArray,
            "/idleness",
            self.onReceiveIdleness,
            100
        )


        print("NEXT NODE: ", self.getNextNode())

        self.get_logger().info(f"PZ agent initialize finished here")


    def onReceiveIdleness(self, msg):
        ''' Called when a new idleness vector is received. '''

        # Update the idleness dictionary.
        for v in range(self.graph.graph.number_of_nodes()):
            self.idleness[v] = msg.data[v]

    def onReceiveTelemetry(self, msg):
        super().onReceiveTelemetry(msg)

        self.agents[msg.sender].position = (msg.odom.pose.pose.position.x,
                                            msg.odom.pose.pose.position.y)

    
    def getNextNode(self):
        ''' Returns the next node to visit. '''

        self.get_logger().info("here is the start of PZ Agent Action")

        obs = self._populateStateSpace("bitmap2", self.agents[self.id], self.all_args.observation_radius, False)
        print("MAKE OBS: ", obs)


        # self.get_logger().info("here is the start of PZ Agent Action process")
        # self.obs_space_new = spaces.Dict(self.t)
        obs = flatten(self.t, obs)
        self.get_logger().info("here is the start of PZ Agent Action process")
        obs = np.array(obs)
        obs = obs.reshape((1,-1))
        self.get_logger().info(f"here is the shape of obs of input rnn {obs.shape}, {self.rnn_states[:,0].shape}, {self.masks[:,0].shape}")
        action, action_log_probs, rnn_state = self.actor(obs, self.rnn_states[:,0], self.masks[:,0], deterministic=True)
        action = action.item()
        self.get_logger().info("here is the mid process")
        self.rnn_states[0,0] = np.array(np.split(_t2n(rnn_state), 1))
        self.get_logger().info(f"the action of PZ Agent Action {action}")
        return action
    

    def _dist(self, a, b):
        ''' Returns the distance between two agents. '''

        return pz_parallel_env._dist(self, a, b)

    # def onAgentAttrition(self, agent):
    #     ''' Called when an agent is lost. '''

    #     super().onAgentAttrition(agent)
    #     if agent == self.id:
    #         return

    #     # Update the Voronoi partitions.
    #     del self.voronoiOrigins[agent]
    #     cell = self.getNodeAllocation(self.voronoiOrigins, self.agentOrigins)
    #     old = self.nodes
    #     self.nodes = self.getNodeOrder(cell)

    #     if old != self.nodes:
    #         self.get_logger().info(f"Updated allocation!\nOld patrol route: {old}\nNew patrol route: {self.nodes}")


    def _populateStateSpace(self, observe_method, agent, radius, allow_done_agents):
        ''' Returns a populated state/observation space.'''

        if radius == None:
            radius = agent.observationRadius

        if allow_done_agents:
            agentList = self.possible_agents
        else:
            agentList = self.agents

        # Calculate the list of visible agents and vertices.
        vertices = [v for v in self.pg.graph.nodes if self._dist(self.pg.getNodePosition(v), agent.position) <= radius]
        agents = [a for a in agentList if self._dist(a.position, agent.position) <= radius]
        for a in agentList:
            if a != agent and a not in agents and self.comms_model.canReceive(a, agent):
                agents.append(a)
                for v in self.pg.graph.nodes:
                    if v not in vertices and self._dist(self.pg.getNodePosition(v), a.position) <= radius:
                        vertices.append(v)
        agents = sorted(agents, key=lambda a: a.id)
        vertices = sorted(vertices)
        
        obs = {}

        # Add agent ID.
        if observe_method in ["ajg_new", "ajg_newer", "adjacency"]:
            obs["agent_id"] = agent.id

        # Add agent position.
        if observe_method in ["ranking", "raw", "old"]:
            obs["agent_state"] = {a: a.position for a in agents}

        # Add vertex idleness time (ranked).
        if observe_method in ["ranking"]:
            nodes_idless = {node : self.idleness[node] for node in vertices}
            unique_sorted_idleness_times = sorted(list(set(nodes_idless.values())))
            obs["vertex_state"] = {v: unique_sorted_idleness_times.index(nodes_idless[v]) for v in vertices}
        
        # Add vertex idleness time (minMax normalized).
        if observe_method in ["ajg_new", "ajg_newer"]:
            # Create numpy array of idleness times.
            idlenessTimes = np.zeros(self.pg.graph.number_of_nodes())
            for v in vertices:
                idlenessTimes[v] = self.idleness[node]
            
            # Normalize.
            if np.size(idlenessTimes) > 0:
                if np.min(idlenessTimes) == np.max(idlenessTimes):
                    idlenessTimes = np.ones(self.pg.graph.number_of_nodes())
                else:
                    idlenessTimes = self._minMaxNormalize(idlenessTimes)

            # Create dictionary with default value of -1.0.
            obs["vertex_state"] = {v: -1.0 for v in range(self.pg.graph.number_of_nodes())}

            # Fill actual values for nodes we can see.
            for v in vertices:
                obs["vertex_state"][v] = idlenessTimes[v]

        # Add vertex idleness time (raw).
        if observe_method in ["raw", "old", "idlenessOnly", "adjacency"]:
            # Create dictionary with default value of -1.0.
            obs["vertex_state"] = {v: -1.0 for v in range(self.pg.graph.number_of_nodes())}

            for node in vertices:
                obs["vertex_state"][node] = self.idleness[node]

        # Add vertex distances from each agent (raw).
        if observe_method in ["old"]:
            vertexDistances = {}
            for a in agents:
                vDists = np.zeros(self.pg.graph.number_of_nodes())
                for v in self.pg.graph.nodes:
                    path = self._getPathToNode(a, v)
                    vDists[v] = self._getAgentPathLength(a, path)
                vertexDistances[a] = vDists
            obs["vertex_distances"] = vertexDistances

        # Add vertex distances from each agent (normalized).
        if observe_method in ["ajg_new", "ajg_newer"]:
            # Calculate the shortest path distances from each agent to each node.
            vDists = np.zeros((len(agents), self.pg.graph.number_of_nodes()))
            for a in agents:
                for v in self.pg.graph.nodes:
                    path = self._getPathToNode(a, v)
                    vDists[a.id, v] = self._getAgentPathLength(a, path)
            
            # Normalize.
            if np.size(vDists) > 0:
                vDists = self._minMaxNormalize(vDists, minimum=0.0, maximum=self.pg.longestPathLength)

            # Convert to dictionary.
            vertexDistances = {}
            for a in agents:
                vertexDistances[a] = vDists[a.id]
            
            obs["vertex_distances"] = vertexDistances

        # Add bitmap observation.
        if observe_method in ["bitmap"]:
            # Create an image which defaults to -1.
            bitmap = -1.0 * np.ones(self.observation_space(agent).shape, dtype=np.float32)

            # Set the observing agent's ID in the (0, 0) position. This is a bit hacky.
            bitmap[0, 0, self.OBSERVATION_CHANNELS.AGENT_ID] = agent.id

            def _normPosition(pos):
                if radius == np.inf:
                    x = self._minMaxNormalize(pos[0], a=0.0, b=self.observe_bitmap_dims[0], minimum=0.0, maximum=self.pg.widthPixels, eps=0.01)
                    y = self._minMaxNormalize(pos[1], a=0.0, b=self.observe_bitmap_dims[1], minimum=0.0, maximum=self.pg.heightPixels, eps=0.01)
                else:
                    x = self._minMaxNormalize(pos[0], a=0.0, b=self.observe_bitmap_dims[0], minimum=agent.position[0] - radius, maximum=agent.position[0] + radius, eps=0.01)
                    y = self._minMaxNormalize(pos[1], a=0.0, b=self.observe_bitmap_dims[1], minimum=agent.position[1] - radius, maximum=agent.position[1] + radius, eps=0.01)
                return x, y

            # Add agents to the observation.
            for a in agents:
                pos = _normPosition(a.position)
                if pos[0] < 0 or pos[0] >= self.observe_bitmap_dims[0] or pos[1] < 0 or pos[1] >= self.observe_bitmap_dims[1]:
                    continue
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.AGENT_ID] = a.id
            
            # Add vertex idleness times to the observation.
            for v in vertices:
                pos = _normPosition(self.pg.getNodePosition(v))
                if pos[0] < 0 or pos[0] >= self.observe_bitmap_dims[0] or pos[1] < 0 or pos[1] >= self.observe_bitmap_dims[1]:
                    continue
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.IDLENESS] = self.idleness[v]
            
            # Add edges to the graph channel.
            for edge in self.pg.graph.edges:
                pos1 = _normPosition(self.pg.getNodePosition(edge[0]))
                pos2 = _normPosition(self.pg.getNodePosition(edge[1]))
                dist = self._dist(pos1, pos2)
                if dist > 0.0:
                    for i in range(int(dist)):
                        pos = (int(pos1[0] + (pos2[0] - pos1[0]) * i / dist), int(pos1[1] + (pos2[1] - pos1[1]) * i / dist))
                        if pos[0] < 0 or pos[0] >= self.observe_bitmap_dims[0] or pos[1] < 0 or pos[1] >= self.observe_bitmap_dims[1]:
                            continue
                        bitmap[pos[0], pos[1], self.OBSERVATION_CHANNELS.GRAPH] = -2.0

            # Add vertices to the graph channel.
            for v in vertices:
                pos = _normPosition(self.pg.getNodePosition(v))
                if pos[0] < 0 or pos[0] >= self.observe_bitmap_dims[0] or pos[1] < 0 or pos[1] >= self.observe_bitmap_dims[1]:
                    continue
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.GRAPH] = v

            obs = bitmap

        # Add bitmap2 observation. This variant uses -1 to indicate unobserved nodes and agents, rather than cropping the bitmap.
        if observe_method in ["bitmap2"]:
            # Create an image which defaults to -1.
            bitmap = -1.0 * np.ones(self.observation_space(agent).shape, dtype=np.float32)

            # Set the observing agent's ID in the (0, 0) position. This is a bit hacky.
            bitmap[0, 0, self.OBSERVATION_CHANNELS.AGENT_ID] = agent.id

            def _normPosition(pos):
                x = self._minMaxNormalize(pos[0], a=0.0, b=self.observe_bitmap_dims[0], minimum=0.0, maximum=self.pg.widthPixels, eps=0.01)
                y = self._minMaxNormalize(pos[1], a=0.0, b=self.observe_bitmap_dims[1], minimum=0.0, maximum=self.pg.heightPixels, eps=0.01)
                return x, y

            # Add agents to the observation.
            for a in agents:
                pos = _normPosition(a.position)
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.AGENT_ID] = a.id
            
            # Add vertex idleness times to the observation.
            for v in vertices:
                pos = _normPosition(self.pg.getNodePosition(v))
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.IDLENESS] = self.idleness[v]
            
            # Add edges to the graph channel.
            for edge in self.pg.graph.edges:
                pos1 = _normPosition(self.pg.getNodePosition(edge[0]))
                pos2 = _normPosition(self.pg.getNodePosition(edge[1]))
                dist = self._dist(pos1, pos2)
                if dist > 0.0:
                    for i in range(int(dist)):
                        pos = (int(pos1[0] + (pos2[0] - pos1[0]) * i / dist), int(pos1[1] + (pos2[1] - pos1[1]) * i / dist))
                        bitmap[pos[0], pos[1], self.OBSERVATION_CHANNELS.GRAPH] = -2.0

            # Add vertices to the graph channel.
            for v in self.pg.graph.nodes:
                pos = _normPosition(self.pg.getNodePosition(v))
                bitmap[int(pos[0]), int(pos[1]), self.OBSERVATION_CHANNELS.GRAPH] = v

            obs = bitmap

        # Add adjacency matrix.
        if observe_method in ["ajg_newer"]:
            # Create adjacency matrix.
            adjacency = -1.0 * np.ones((self.pg.graph.number_of_nodes(), self.pg.graph.number_of_nodes()), dtype=np.float32)
            for edge in self.pg.graph.edges:
                adjacency[edge[0], edge[1]] = 1.0
                adjacency[edge[1], edge[0]] = 1.0
            obs["adjacency"] = adjacency

        # Add weighted adjacency matrix (normalized).
        if observe_method in ["adjacency"]:
            # Create adjacency matrix.
            adjacency = -1.0 * np.ones((self.pg.graph.number_of_nodes(), self.pg.graph.number_of_nodes()), dtype=np.float32)
            for edge in self.pg.graph.edges:
                maxWeight = max([self.pg.graph.edges[e]["weight"] for e in self.pg.graph.edges])
                minWeight = min([self.pg.graph.edges[e]["weight"] for e in self.pg.graph.edges])
                weight = self._minMaxNormalize(self.pg.graph.edges[edge]["weight"], minimum=minWeight, maximum=maxWeight)
                adjacency[edge[0], edge[1]] = weight
                adjacency[edge[1], edge[0]] = weight
            obs["adjacency"] = adjacency
        
        # Add agent graph position vector.
        if observe_method in ["adjacency", "ajg_newer"]:
            graphPos = {}
            # Set default value of -1.0
            for a in self.possible_agents:
                graphPos[a] = -1.0 * np.ones(3, dtype=np.float32)
            
            # Fill in actual values for agents we can see.
            for a in agents:
                vec = np.zeros(3, dtype=np.float32)
                if a.edge == None:
                    vec[0] = a.lastNode
                    vec[1] = a.lastNode
                    vec[2] = 1.0
                else:
                    vec[0] = a.edge[0]
                    vec[1] = a.edge[1]
                    vec[2] = self._getAgentPathLength(a, self._getPathToNode(a, a.edge[0])) / self.pg.graph.edges[a.edge]["weight"]
                graphPos[a] = vec
            obs["agent_graph_position"] = graphPos
        
        if (type(obs) == dict and obs == {}) or (type(obs) != dict and len(obs) < 1):
            raise ValueError(f"Invalid observation method {self.observe_method}")
        
        return obs
    
    def _minMaxNormalize(self, x, eps=1e-8, a=0.0, b=1.0, maximum=None, minimum=None):
        ''' Normalizes numpy array x to be between a and b. '''

        if maximum is None:
            maximum = np.max(x)
        if minimum is None:
            minimum = np.min(x)
        return a + (x - minimum) * (b - a) / (maximum - minimum + eps)



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