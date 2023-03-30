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
import random
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
        self.declare_parameter("output_file", "./results.zarr")
        self.declare_parameter("initial_poses", [0.0, 0.0])
        self.declare_parameter("attrition_times", "")
        self.declare_parameter("agent_count", 1)
        self.declare_parameter("runtime", 0)
        self.algorithm = self.get_parameter("algorithm_name").get_parameter_value().string_value
        self.map = self.get_parameter("map").get_parameter_value().string_value
        self.graphFilePath = self.get_parameter("patrol_graph_file").get_parameter_value().string_value
        self.outputFilePath = self.get_parameter("output_file").get_parameter_value().string_value
        self.initialPoses = self.get_parameter("initial_poses").get_parameter_value().double_array_value
        self.attritionTimes = self.get_parameter("attrition_times").get_parameter_value().string_value
        self.agent_count = self.get_parameter("agent_count").get_parameter_value().integer_value
        self.runtime = self.get_parameter("runtime").get_parameter_value().integer_value

        if len(self.attritionTimes) > 0:
            self.attritionTimes = [float(i) for i in self.attritionTimes.split(",")]
        else:
            self.attritionTimes = []

        self.get_logger().info(f"Initializing monitor for {self.agent_count} agents on map {self.map}.")

        random.seed(42)

        # Variables.
        self.experimentInitialized = False
        self.agentInitialized = np.zeros(self.agent_count)
        self.visitTimes = []
        self.visitAgents = []
        self.visitNodes = []
        self.commsTimes = []
        self.attritionList = []
        self.agentsRemaining = set(range(self.agent_count))

        # Components.
        self.graph = PatrolGraph(self.graphFilePath)
        self.agentOrigins = self.graph.getOriginsFromInitialPoses(self.initialPoses)
        self.agentPositions = [(0.0, 0.0) for a in range(self.agent_count)]

        # Data storage.
        self.zarrRoot = zarr.open(self.outputFilePath, mode="a")
        self.zarrRoot2 = self.zarrRoot.require_group(
            f"{self.map}_{self.agent_count}"
        ).require_group(
            f"{self.algorithm}"
        )
        self.zarrData = self.zarrRoot2.require_group(
            f"run_{len(self.zarrRoot2) + 1}"
        )
        self.zarrData["datetime"] = [f"{datetime.datetime.now().isoformat()}"]
        self.zarrData["graph"] = nx.to_numpy_array(self.graph.graph)
        self.zarrData["origins"] = self.agentOrigins
        self.zarrData["runtime_requested"] = [self.runtime]
        self.zarrData.require_group("visits")

        # Subscribers.
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        # self.subTelemetry = self.create_subscription(
        #     AgentTelemetry,
        #     "/positions",
        #     self.onReceiveTelemetry,
        #     100
        # )
        self.subResults = self.create_subscription(
            Int16MultiArray,
            "/results",
            self.onReceiveResults,
            100
        )

        # Publishers.
        # self.pubTelemetry = self.create_publisher(
        #     AgentTelemetry,
        #     "/positions",
        #     100
        # )
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
        self.zarrData["comm_times"] = self.commsTimes
        self.zarrData["attrition_times"] = self.attritionList

        timeElapsed = self.get_clock().now() - self.timeStart
        self.zarrData["runtime_actual"] = [timeElapsed.nanoseconds]



    def onExperimentInitialized(self):
        ''' Begins execution after experiment initialization completes. '''

        self.timeStart = self.get_clock().now()
        self.experimentInitialized = True
        self.get_logger().info("Initialization complete.")


        # Notify agents that we are ready.
        msg = Int16MultiArray()
        msg.data = [-1, self.MSG_TYPES["INITIALIZE_MSG_TYPE"], 100]
        self.pubResults.publish(msg)

        # Start timer.
        if self.runtime > 0:
            self.timerStopSim = self.create_timer(
                float(self.runtime), # period (seconds)
                self.onTimerStopSim
            )
        if len(self.attritionTimes) > 0:
            duration = (self.timeStart + rclpy.duration.Duration(seconds=self.attritionTimes[0])) - self.get_clock().now()
            duration = duration.nanoseconds / 1.0e9
            self.timerAttrition = self.create_timer(
                duration, # period (seconds)
                self.onTimerAttrition
            )


    def onReceiveResults(self, msg):
        ''' Called when results are received. '''

        sender = msg.data[0]
        msgType = msg.data[1]
        if sender != -1:
            if msgType == self.MSG_TYPES["INITIALIZE_MSG_TYPE"]:
                self.agentInitialized[sender] = msg.data[2]

            elif msgType == self.MSG_TYPES["TARGET_REACHED_MSG_TYPE"]:
                self.onAgentReachedNode(sender, msg.data[2])
            
            elif self.experimentInitialized and msgType >= 20:
                # count the algorithm messages.
                timeElapsed = self.get_clock().now() - self.timeStart
                self.commsTimes.append(timeElapsed.nanoseconds)


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
    
    def onTimerStopSim(self):
        ''' Stops the sim after time limit. '''

        self.get_logger().warn(f"Simulation completed after {self.runtime}s. Shutting down.")
        raise KeyboardInterrupt()
    
    def onTimerAttrition(self):
        ''' Kills agent after time limit. '''

        del self.attritionTimes[0]

        agent = random.sample(list(self.agentsRemaining), 1)[0]
        self.agentsRemaining -= {agent}

        self.perfromAgentAttrition(agent)

        self.timerAttrition.cancel()
        self.timerAttrition = None
        if len(self.attritionTimes) > 0:
            duration = (self.timeStart + rclpy.duration.Duration(seconds=self.attritionTimes[0])) - self.get_clock().now()
            duration = duration.nanoseconds / 1.0e9
            self.timerAttrition = self.create_timer(
                duration, # period (seconds)
                self.onTimerAttrition
            )


    def perfromAgentAttrition(self, agent):
        self.get_logger().warn(f"Performing attrition on agent {agent}.")
        os.system(f"pkill -2 -f '__ns:=/agent{agent}'")

        timeElapsed = self.get_clock().now() - self.timeStart
        self.attritionList.append([timeElapsed.nanoseconds, agent])


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

    rclpy.shutdown()
if __name__ == '__main__':
    main()
