from ast import literal_eval
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data, qos_profile_parameters

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from patrolling_sim_interfaces.msg import AgentTelemetry
from std_msgs.msg import Int16MultiArray

from patrol_algorithms_ahpa.PatrolGraph import PatrolGraph

class BasePatrolAgent(Node):
    MSG_TYPES = {
        "INITIALIZE_MSG_TYPE": 10,
        "TARGET_REACHED_MSG_TYPE": 11,
        "INTERFERENCE_MSG_TYPE": 12,
        "AGENT_ATTRITION_MSG_TYPE": 13
    }

    def __init__(self):
        super().__init__("PatrolAgent")
        
        # Parameters.
        self.declare_parameter("id_robot", 0)
        self.declare_parameter("patrol_graph_file", "/home/anthony/idev/patrolling_sim/src/patrolling_sim/models/maps/cumberland/cumberland.graph")
        self.declare_parameter("initial_poses", "[0.0, 0.0]")
        self.declare_parameter("initial_pos.x", 0.0)
        self.declare_parameter("initial_pos.y", 0.0)
        self.declare_parameter("goal_reached_wait", 3.0)
        self.declare_parameter("communication_delay", 0.2)
        self.declare_parameter("lost_message_rate", 0.0)
        self.declare_parameter("agent_count", 1)
        self.declare_parameter("tf_prefix", "")
        self.graphFilePath = self.get_parameter("patrol_graph_file").get_parameter_value().string_value
        self.initialPoses = literal_eval(self.get_parameter("initial_poses").get_parameter_value().string_value)
        self.goal_reached_wait = self.get_parameter("goal_reached_wait").get_parameter_value().double_value
        self.communication_delay = self.get_parameter("communication_delay").get_parameter_value().double_value
        self.lost_message_rate = self.get_parameter("lost_message_rate").get_parameter_value().double_value
        self.agent_count = self.get_parameter("agent_count").get_parameter_value().integer_value
        self.tf_prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value
        self.id = self.get_parameter("id_robot").get_parameter_value().integer_value

        self.get_logger().info(f"Initializing patrol agent {self.id}.")

        # Variables.
        self.experimentInitialized = False

        # Components.
        self.graph = PatrolGraph(self.graphFilePath)
        self.agentOrigins = self.graph.getOriginsFromInitialPoses(self.initialPoses)
        self.agentPositions = [(0.0, 0.0) for a in range(self.agent_count)]

        # Subscribers.
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.subTelemetry = self.create_subscription(
            AgentTelemetry,
            "/positions",
            self.onReceiveTelemetry,
            qos_profile_sensor_data
        )
        self.subOdom = self.create_subscription(
            Odometry,
            "odom",
            self.onReceiveOdometry,
            qos_profile_sensor_data
        )
        self.subResults = self.create_subscription(
            Int16MultiArray,
            "/results",
            self.onReceiveResults,
            qos_profile_parameters
        )

        # Publishers.
        self.pubTelemetry = self.create_publisher(
            AgentTelemetry,
            "/positions",
            qos_profile_sensor_data
        )
        self.pubResults = self.create_publisher(
            Int16MultiArray,
            "/results",
            qos_profile_parameters
        )

        # Action clients.
        self.acNav2Pose = ActionClient(self,
            NavigateToPose,
            'navigate_to_pose'
        )


        # Wait for resources.
        self.acNav2Pose.wait_for_server()
        self._waitForNodeToActivate("bt_navigator")
        self._waitForNodeToActivate("amcl")
        while rclpy.ok() and not self.tfBuffer.can_transform(self.tf_prefix + "map", self.tf_prefix + "base_link", rclpy.time.Time()):
            rclpy.spin_once(self)
        

        # Setup is complete.
        self.get_logger().info("Initialization complete.")
    
        # Timers.
        # These begin executing immediately.
        self.timerSendTelemetry = self.create_timer(
            1.0, # period (seconds)
            self.onTimerSendTelemetry
        )
        self.timerAdvertizeReady = self.create_timer(
            1.0, # period (seconds)
            self.onTimerAdvertizeReady
        )


    def onExperimentInitialized(self):
        ''' Begins execution after experiment initialization completes. '''

        self.experimentInitialized = True
        self.get_logger().info("Let's Patrol!")

        self.goToNode(self.getNextNode())


    def onTimerAdvertizeReady(self):
        ''' Initializes the node and communicates with controller.
            Part of the experiment setup process. Waits for
            experiment initialization. '''
        
        if self.experimentInitialized:
            # Stop advertizing after for initialization.
            self.timerAdvertizeReady.cancel()
        else:
            # Send advertizement.
            msg = Int16MultiArray()
            msg.data = [
                self.id,
                self.MSG_TYPES['INITIALIZE_MSG_TYPE'],
                1
            ]
            self.pubResults.publish(msg)


    def onReceiveResults(self, msg):
        ''' Called when results are received. '''

        sender = msg.data[0]
        msgType = msg.data[1]
        if sender == -1:
            if msgType == self.MSG_TYPES["INITIALIZE_MSG_TYPE"]:
                if not self.experimentInitialized:
                    self.onExperimentInitialized()
            elif msgType == self.MSG_TYPES["AGENT_ATTRITION_MSG_TYPE"]:
                agentIdx = msg.data[2]

                if agentIdx == self.id:
                    # Shut ourselves down.
                    self.get_logger().info(f"Shutting down agent {self.id}")
                    os.system(f"pkill -2 -f '__ns:=/agent{self.id}'")
                else:
                    # Handle someone else's shutdown.
                    self.onAgentAttrition(agentIdx)

    def onReceiveOdometry(self, msg):
        ''' Called when odometry information is received.
            Use this for positioning if simulation has no error. '''

        self.agentPositions[self.id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def onTimerGetPosition(self):
        ''' Called periodically to look up current position. '''

        raise NotImplementedError()
        try:
            t = self.tfBuffer.lookupTransform(
                self.tf_prefix + "base_link",
                self.tf_prefix + "map",
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().error(f"Could not look up transformation between map and base_link.")
            return
        
        self.agentPositions[self.id] = (
            t.transform.translation.x,
            t.transform.translation.y
        )

    def onReceiveTelemetry(self, msg):
        if msg.sender != self.id:
            self.agentPositions[msg.sender] = (
                msg.odom.pose.pose.position.x,
                msg.odom.pose.pose.position.y
            )
    
    def onTimerSendTelemetry(self):
        ''' Periodically sends agent telemetry. '''

        msg = AgentTelemetry()
        msg.sender = self.id
        msg.odom.header.frame_id = f"agent{self.id}/map"
        msg.odom.pose.pose.position.x = self.agentPositions[self.id][0]
        msg.odom.pose.pose.position.y = self.agentPositions[self.id][1]
        self.pubTelemetry.publish(msg)

    def onAgentAttrition(self, agent):
        pass

    def onNav2PoseGoalResponse(self, future):
        ''' Called when the action server responds to our request. '''

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected! Something is very wrong.")
            return

        self.get_logger().info("Navigation goal accepted.")

        self.futureNav2PoseResult = goal_handle.get_result_async()
        self.futureNav2PoseResult.add_done_callback(self.onNav2PoseResult)

    def onNav2PoseResult(self, future):
        ''' Called when navigation has completed. '''

        # Check navigation status.
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.onNavigationGoalSuccess()
        else:
            self.onNavigationGoalFailed(status)
    
    def onNavigationGoalFailed(self, status):
        ''' Called when we did not reach the navigation goal. '''

        self.get_logger().error(f"Navigation failed with status code: {status}. Retrying.")
        self.goToNode(self.goalNode)

    def onNavigationGoalSuccess(self):
        ''' Called when we have successfully navigated to a node. '''

        # Notify the monitor.
        msg = Int16MultiArray()
        msg.data = [self.id, self.MSG_TYPES["TARGET_REACHED_MSG_TYPE"], self.goalNode]
        self.pubResults.publish(msg)

        # Go to next goal.
        self.goToNode(self.getNextNode())

    def goToNode(self, node):
        ''' Orders the agent to proceed to the next node. '''

        self.goalNode = node
        position = self.graph.getNodePosition(node)
        
        self.get_logger().info(f"Requesting navigation goal for node {node} {position}.")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position[0])
        goal.pose.pose.position.y = float(position[1])
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        goal.behavior_tree = ""
        
        self.futureNav2PoseGoal = self.acNav2Pose.send_goal_async(goal)
        self.futureNav2PoseGoal.add_done_callback(self.onNav2PoseGoalResponse)

    def getNextNode(self):
        ''' Called to determine which node the agent should travel to next.
            This should be implemented by subclasses.'''

        raise NotImplementedError("Override this in your own subclass.")

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.get_logger().debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.get_logger().debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().debug(f'Result of get_state: {state}')
            time.sleep(2)
        return


def main(args=None):
    # Initialize ROS2.
    rclpy.init(args=args)

    # Create the node and execute.
    agent = BasePatrolAgent()
    rclpy.spin(agent)

    # Destroy the node explicitly.
    agent.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
