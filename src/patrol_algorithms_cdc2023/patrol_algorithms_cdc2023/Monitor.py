import datetime
import networkx as nx
import numpy as np
import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import zarr

from patrolling_sim_interfaces.msg import AgentTelemetry
from std_msgs.msg import Int16MultiArray

from patrol_algorithms_cdc2023.PatrolGraph import PatrolGraph

class MonitorNode(Node):
    MSG_TYPES = {
        "INITIALIZE_MSG_TYPE": 10,
        "TARGET_REACHED_MSG_TYPE": 11,
        "INTERFERENCE_MSG_TYPE": 12,
        "AGENT_ATTRITION_MSG_TYPE": 13
    }

    def __init__(self):
        super().__init__("PatrolAgent")
        
        # Parameters.
        self.declare_parameter("algorithm_name", "Random")
        self.declare_parameter("map", "cumberland")
        self.declare_parameter("patrol_graph_file", "/home/anthony/dev/patrolling_sim/src/patrolling_sim/models/maps/cumberland/cumberland.graph")
        self.declare_parameter("initial_poses", [0.0, 0.0])
        self.declare_parameter("agent_count", 1)
        self.algorithm = self.get_parameter("algorithm_name").get_parameter_value().string_value
        self.map = self.get_parameter("map").get_parameter_value().string_value
        self.graphFilePath = self.get_parameter("patrol_graph_file").get_parameter_value().string_value
        self.initialPoses = self.get_parameter("initial_poses").get_parameter_value().double_array_value
        self.agent_count = self.get_parameter("agent_count").get_parameter_value().integer_value

        self.get_logger().info(f"Initializing monitor for {self.agent_count} agents on map {self.map}.")

        # Variables.
        self.experimentInitialized = False
        self.agentInitialized = np.zeros(self.agent_count)
        self.visitTimes = []
        self.visitAgents = []
        self.visitNodes = []

        # Components.
        self.graph = PatrolGraph(self.graphFilePath)
        self.agentOrigins = self.graph.getOriginsFromInitialPoses(self.initialPoses)
        self.agentPositions = [(0.0, 0.0) for a in range(self.agent_count)]

        # Data storage.
        self.zarrRoot = zarr.open("./results.zarr", mode="a")
        self.zarrRoot2 = self.zarrRoot.require_group(
            f"{self.map}_{self.agent_count}"
        ).require_group(
            f"{self.algorithm}"
        )
        self.zarrData = self.zarrRoot2.require_group(
            f"run_{len(self.zarrRoot2) + 1}"
        )
        self.zarrData["datetime"] = f"{datetime.datetime.now().isoformat()}"
        self.zarrData["graph"] = nx.to_numpy_array(self.graph.graph)
        self.zarrData.require_group("visits")

        # Subscribers.
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.subTelemetry = self.create_subscription(
            AgentTelemetry,
            "/positions",
            self.onReceiveTelemetry,
            100
        )
        self.subResults = self.create_subscription(
            Int16MultiArray,
            "/results",
            self.onReceiveResults,
            100
        )

        # Publishers.
        self.pubTelemetry = self.create_publisher(
            AgentTelemetry,
            "/positions",
            100
        )
        self.pubResults = self.create_publisher(
            Int16MultiArray,
            "/results",
            100
        )

        # Wait for initialization.
        self.waitForAgents()

        # Setup is complete.
        self.onExperimentInitialized()

    def writeData(self):
        ''' Called upon conclusion to write data. '''

        self.zarrData["visits"]["time_ns"] = self.visitTimes
        self.zarrData["visits"]["agent"] = self.visitAgents
        self.zarrData["visits"]["node"] = self.visitNodes


    def onExperimentInitialized(self):
        ''' Begins execution after experiment initialization completes. '''

        self.experimentInitialized = True
        self.get_logger().info("Initialization complete.")

        self.timeStart = self.get_clock().now()

        # Notify agents that we are ready.
        msg = Int16MultiArray()
        msg.data = [-1, self.MSG_TYPES["INITIALIZE_MSG_TYPE"], 100]
        self.pubResults.publish(msg)


    def onReceiveResults(self, msg):
        ''' Called when results are received. '''

        sender = msg.data[0]
        msgType = msg.data[1]
        if sender != -1:
            if msgType == self.MSG_TYPES["INITIALIZE_MSG_TYPE"]:
                self.agentInitialized[sender] = msg.data[2]

            elif msgType == self.MSG_TYPES["TARGET_REACHED_MSG_TYPE"]:
                self.onAgentReachedNode(sender, msg.data[2])


    def onReceiveTelemetry(self, msg):
        self.agentPositions[msg.sender] = (
            msg.odom.pose.pose.position.x,
            msg.odom.pose.pose.position.y
        )

    def onAgentReachedNode(self, agent, node):
        ''' Called when agent reaches a node. Record data here. '''

        timeElapsed = self.get_clock().now() - self.timeStart
        self.visitTimes.append(timeElapsed.nanoseconds)
        self.visitAgents.append(agent)
        self.visitNodes.append(node)

    def waitForAgents(self):
        ''' Blocks until all agents report ready. '''

        while np.sum(self.agentInitialized) < self.agent_count:
            rclpy.spin_once(self)


def main(args=None):
    # Initialize ROS2.
    rclpy.init(args=args)

    # Create the node and execute.
    monitor = MonitorNode()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.writeData()

    # Destroy the node explicitly.
    monitor.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
