from ast import literal_eval
import datetime
import networkx as nx
import numpy as np
import os
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameters
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import random
import zarr

from patrolling_sim_interfaces.msg import AgentTelemetry
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from flatland_msgs.srv import DeleteModel

from patrol_algorithms_magec.PzEnvironment import PzEnvironment
from patrol_algorithms_ahpa.PatrolGraph import PatrolGraph


class MonitorNode(Node):
    MSG_TYPES = {
        "INITIALIZE_MSG_TYPE": 10,
        "TARGET_REACHED_MSG_TYPE": 11,
        "INTERFERENCE_MSG_TYPE": 12,
        "AGENT_ATTRITION_MSG_TYPE": 13
    }

    TIMEOUT_AGENT_ONLINE_NS = 2e9 #nanoseconds

    def __init__(self):
        super().__init__("Monitor")
        
        # Parameters.
        self.declare_parameter("algorithm_name", "Random")
        self.declare_parameter("map", "cumberland")
        self.declare_parameter("patrol_graph_file", "/home/anthony/dev/patrolling_sim/src/patrolling_sim/models/maps/cumberland/cumberland.graph")
        self.declare_parameter("output_file", "./results.zarr")
        self.declare_parameter("initial_poses", "[0.0, 0.0]")
        self.declare_parameter("attrition_times", "")
        self.declare_parameter("agent_count", 1)
        self.declare_parameter("runtime", 0)
        self.declare_parameter("startup_timeout", 100.0)
        self.algorithm = self.get_parameter("algorithm_name").get_parameter_value().string_value
        self.map = self.get_parameter("map").get_parameter_value().string_value
        self.graphFilePath = self.get_parameter("patrol_graph_file").get_parameter_value().string_value
        self.outputFilePath = self.get_parameter("output_file").get_parameter_value().string_value
        self.initialPoses = literal_eval(self.get_parameter("initial_poses").get_parameter_value().string_value)
        self.attritionTimes = self.get_parameter("attrition_times").get_parameter_value().string_value
        self.agent_count = self.get_parameter("agent_count").get_parameter_value().integer_value
        self.runtime = self.get_parameter("runtime").get_parameter_value().integer_value
        self.startupTimeout = self.get_parameter("startup_timeout").get_parameter_value().double_value

        self.attritionTimes = [float(i) for i in self.attritionTimes.split(",") if len(i) > 0]
        if self.attritionTimes[0] < 0:
            self.attritionTimes = []

        self.get_logger().info(f"Initializing monitor for {self.agent_count} agents on map {self.map}.")

        random.seed(100)

        # Variables.
        self.experimentInitialized = False
        self.agentInitialized = np.zeros(self.agent_count)
        self.visitTimes = []
        self.visitAgents = []
        self.visitNodes = []
        self.commsTimes = []
        self.attritionList = []
        self.agentsRemaining = list(range(self.agent_count))
        self.agentLastTelemetryTime = [self.get_clock().now() for _ in range(self.agent_count)]
        self.onlineAgentsPrev = []

        # Components.
        self.graph = PatrolGraph(self.graphFilePath)
        self.agentOrigins = self.graph.getOriginsFromInitialPoses(self.initialPoses)
        self.agentPositions = list(zip(self.initialPoses[0::2], self.initialPoses[1::2]))

        # Create the PzEnvironment.
        self.pzEnv = PzEnvironment(self)

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
        self.subTelemetry = self.create_subscription(
            AgentTelemetry,
            "/positions",
            self.onReceiveTelemetry,
            qos_profile_sensor_data
        )
        self.subResults = self.create_subscription(
            Int16MultiArray,
            "/results",
            self.onReceiveResults,
            qos_profile_parameters
        )

        # Publishers.
        self.pubResults = self.create_publisher(
            Int16MultiArray,
            "/results",
            qos_profile_parameters
        )
        self.pubIdleness = self.create_publisher(
            Float32MultiArray,
            "/idleness",
            qos_profile_sensor_data
        )
        self.pubMarkerNodes = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            10
        )
        self.pubMarkerEdges = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            10
        )

        # Service clients.
        self.get_logger().info("Waiting for /delete_model service...")
        self.deleteModelClient = self.create_client(DeleteModel, "/delete_model")
        if not self.deleteModelClient.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Service /delete_model not available. Agents will not be removed from simulation upon attrition!")
            self.deleteModelClient = None
        self.get_logger().info("Service /delete_model available.")

        # Create timers.
        self.timerSendMarkers = self.create_timer(
            2.0, # period (seconds)
            self.onTimerSendMarkers
        )
        self.timerReportOnlineAgents = self.create_timer(
            5.0, # period (seconds)
            self.onTimerReportOnlineAgents
        )
        self.clockSystem = Clock()
        self.timerFailedStartup = self.create_timer(
            self.startupTimeout, # period (seconds)
            self.onTimerFailedStartup,
            clock = self.clockSystem
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

        # Cancel the failed startup timer.
        self.timerFailedStartup.cancel()
        self.timerFailedStartup = None

        # Start timer.
        self.timerSendIdleness = self.create_timer(
            0.1, # period (seconds)
            self.onTimerSendIdleness
        )
        self.timerInitMsg = self.create_timer(
            1.0, # period (seconds)
            self.onTimerSendInitialize
        )
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
        
        self.pzEnv.onExperimentInitialized()


    def onReceiveResults(self, msg):
        ''' Called when results are received. '''

        sender = msg.data[0]
        msgType = msg.data[1]
        if sender != -1:
            if msgType == self.MSG_TYPES["INITIALIZE_MSG_TYPE"]:
                if self.agentInitialized[sender] != 1 and msg.data[2] == 1:
                    self.get_logger().info(f"Agent {sender} initialized.")
                if self.agentInitialized[sender] == 1 and msg.data[2] != 1:
                    self.get_logger().warn(f"Agent {sender} is no longer ready!")
                self.agentInitialized[sender] = msg.data[2]

            elif msgType == self.MSG_TYPES["TARGET_REACHED_MSG_TYPE"]:
                self.onAgentReachedNode(sender, msg.data[2])
            
        if self.experimentInitialized and msgType >= 13:
            # count the algorithm messages.
            timeElapsed = self.get_clock().now() - self.timeStart
            self.commsTimes.append(timeElapsed.nanoseconds)


    def onReceiveTelemetry(self, msg):
        self.agentPositions[msg.sender] = (
            msg.odom.pose.pose.position.x,
            msg.odom.pose.pose.position.y
        )
        self.agentLastTelemetryTime[msg.sender] = self.get_clock().now()
        self.pzEnv.onReceiveTelemetry(msg)

    def onAgentReachedNode(self, agent, node):
        ''' Called when agent reaches a node. Record data here. '''

        timeElapsed = self.getTimeElapsed()
        self.visitTimes.append(timeElapsed.nanoseconds)
        self.visitAgents.append(agent)
        self.visitNodes.append(node)
        self.graph.setNodeVisitTime(node, timeElapsed.nanoseconds / 1.0e9)
        self.get_logger().info(f"Agent {agent} reached node {node} at time {timeElapsed.nanoseconds / 1.0e9}.")
        self.pzEnv.onNavigationGoalSuccess(agent, node)
    
    def onTimerSendInitialize(self):
        ''' Repeatedly send the initialization message. '''

        # Notify agents that we are ready.
        msg = Int16MultiArray()
        msg.data = [-1, self.MSG_TYPES["INITIALIZE_MSG_TYPE"], 100]
        self.pubResults.publish(msg)

    def onTimerStopSim(self):
        ''' Stops the sim after time limit. '''

        self.get_logger().warn(f"Simulation completed after {self.runtime}s. Shutting down.")
        raise KeyboardInterrupt()
    

    def onTimerFailedStartup(self):
        ''' Shuts down the simulation if startup times out. '''

        online = self.getOnlineAgents()
        self.get_logger().error(f"Startup failed. Shutting down. Current agents online: {online}")
        raise KeyboardInterrupt()


    def onTimerSendIdleness(self):
        ''' Repeatedly send the idleness message. '''

        secondsElapsed = (self.get_clock().now() - self.timeStart).nanoseconds / 1.0e9 #seconds to ns

        msg = Float32MultiArray()
        msg.data = [self.graph.getNodeIdlenessTime(node, secondsElapsed) for node in range(self.graph.graph.number_of_nodes())]
        self.pubIdleness.publish(msg)
    
    def onTimerSendMarkers(self):
        ''' Publishes Marker information for rviz. '''

        msg = MarkerArray()

        # Create node marker.
        nodes = Marker()
        nodes.header.frame_id = "map"
        nodes.header.stamp = self.get_clock().now().to_msg()
        nodes.ns = "nodes"
        nodes.id = 0
        nodes.type = Marker.POINTS
        nodes.action = Marker.ADD
        nodes.pose.orientation.w = 1.0
        nodes.scale.x = 0.1
        nodes.scale.y = 0.1
        nodes.color.r = 1.0
        nodes.color.a = 1.0
        nodes.points = []
        for node in self.graph.graph.nodes():
            pos = self.graph.getNodePosition(node)
            p = Point()
            p.x = pos[0]
            p.y = pos[1]
            p.z = 0.1
            nodes.points.append(p)
        msg.markers.append(nodes)

        # Create edge marker.
        edges = Marker()
        edges.header.frame_id = "map"
        edges.header.stamp = self.get_clock().now().to_msg()
        edges.ns = "edges"
        edges.id = 0
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.pose.orientation.w = 1.0
        edges.scale.x = 0.05
        edges.color.g = 1.0
        edges.color.a = 1.0
        edges.points = []
        for edge in self.graph.graph.edges():
            pos1 = self.graph.getNodePosition(edge[0])
            pos2 = self.graph.getNodePosition(edge[1])
            p1 = Point()
            p1.x = pos1[0]
            p1.y = pos1[1]
            p1.z = 0.1
            p2 = Point()
            p2.x = pos2[0]
            p2.y = pos2[1]
            p2.z = 0.1
            edges.points.append(p1)
            edges.points.append(p2)
        msg.markers.append(edges)

        # Publish the markers.
        self.pubMarkerNodes.publish(msg)


    def onTimerAttrition(self):
        ''' Kills agent after time limit. '''

        del self.attritionTimes[0]

        agent = self.agentsRemaining[0]
        self.agentsRemaining.remove(agent)

        self.performAgentAttrition(agent)

        self.timerAttrition.cancel()
        self.timerAttrition = None
        if len(self.attritionTimes) > 0:
            duration = (self.timeStart + rclpy.duration.Duration(seconds=self.attritionTimes[0])) - self.get_clock().now()
            duration = duration.nanoseconds / 1.0e9
            self.timerAttrition = self.create_timer(
                duration, # period (seconds)
                self.onTimerAttrition
            )


    def onTimerReportOnlineAgents(self):
        ''' Reports online agents. '''

        online = self.getOnlineAgents()
        if online != self.onlineAgentsPrev:
            self.get_logger().info(f"Online agents: {online}")
            self.onlineAgentsPrev = online

    def getTimeElapsed(self):
        ''' Returns the time elapsed since the start of the experiment. '''

        return self.get_clock().now() - self.timeStart

    def getOnlineAgents(self):
        ''' Returns a list of online agents. '''

        online = []
        timeNow = self.get_clock().now()
        for agent in range(self.agent_count):
            if (timeNow - self.agentLastTelemetryTime[agent]).nanoseconds < self.TIMEOUT_AGENT_ONLINE_NS:
                online.append(agent)
        return online

    def performAgentAttrition(self, agent):
        self.get_logger().warn(f"Performing attrition on agent {agent}.")
        os.system(f"pkill -2 -f '__ns:=/agent{agent}'")

        msg = Int16MultiArray()
        msg.data = [-1, self.MSG_TYPES["AGENT_ATTRITION_MSG_TYPE"], agent]
        self.pubResults.publish(msg)

        timeElapsed = self.get_clock().now() - self.timeStart
        self.attritionList.append([timeElapsed.nanoseconds, agent])

        # Remove from simulation.
        self.pzEnv.onAgentAttrition(agent)
        if self.deleteModelClient is not None:
            req = DeleteModel.Request()
            req.name = f"agent{agent}"
            future = self.deleteModelClient.call_async(req)


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
