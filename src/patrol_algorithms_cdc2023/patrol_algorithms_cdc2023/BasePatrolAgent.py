import rclpy
from rclpy.node import Node

from patrolling_sim_interfaces.msg import AgentTelemetry
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray

from patrol_algorithms_cdc2023.PatrolGraph import PatrolGraph

class BasePatrolAgent(Node):
    def __init__(self):
        super().__init__("PatrolAgent")
        
        # Parameters.
        self.declare_parameter("id_robot", 0)
        self.declare_parameter("patrol_graph_file", "/home/anthony/idev/patrolling_sim/src/patrolling_sim/models/maps/cumberland/cumberland.graph")
        self.declare_parameter("initial_poses", [0.0, 0.0])
        self.declare_parameter("initial_pos.x", 0.0)
        self.declare_parameter("initial_pos.y", 0.0)
        self.declare_parameter("goal_reached_wait", 3.0)
        self.declare_parameter("communication_delay", 0.2)
        self.declare_parameter("lost_message_rate", 0.0)
        self.declare_parameter("agent_count", 1)
        self.graphFilePath = self.get_parameter("patrol_graph_file").get_parameter_value().string_value
        self.initialPoses = self.get_parameter("initial_poses").get_parameter_value().double_array_value
        self.goal_reached_wait = self.get_parameter("goal_reached_wait").get_parameter_value().double_value
        self.communication_delay = self.get_parameter("communication_delay").get_parameter_value().double_value
        self.lost_message_rate = self.get_parameter("lost_message_rate").get_parameter_value().double_value
        self.agent_count = self.get_parameter("agent_count").get_parameter_value().integer_value
        self.id = self.get_parameter("id_robot").get_parameter_value().integer_value

        self.get_logger().info(f"My ID: {self.id}")

        self.get_logger().info(f"My initial pose: ({self.initialPoses[self.id * 2]}, {self.initialPoses[self.id * 2 + 1]}), All initial poses: {self.initialPoses}")

        # Components.
        self.graph = PatrolGraph(self.graphFilePath)
        self.agentOrigins = self.graph.getOriginsFromInitialPoses(self.initialPoses)
        self.agentPositions = [(0.0, 0.0) for a in range(self.agent_count)]

        # Subscribers.
        self.subTelemetry = self.create_subscription(
            AgentTelemetry,
            "/positions",
            self.onReceiveTelemetry,
            100
        )
        self.subOdom = self.create_subscription(
            Odometry,
            "odom",
            self.onReceiveOdometry,
            100
        )
        self.subResults = self.create_subscription(
            Int16MultiArray,
            "/results",
            self.onReceiveResults,
            100
        )

        # Publishers.

    # def onReceiveOdometry(self, msg):
    #     self.agentPositions[self.id] = (

    #     )

    def onReceiveTelemetry(self, msg):
        if msg.sender != self.id:
            self.agentPositions[msg.sender] = (
                msg.odom.pose.pose.position.x,
                msg.odom.pose.pose.position.y
            )
    
    def onTimerSendTelemetry(self, event):
        msg = AgentTelemetry()
        msg.sender = self.id
        raise NotImplementedError()


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
