import rclpy

from patrol_algorithms_cdc2023.BasePatrolAgent import BasePatrolAgent


class AhpaAgent(BasePatrolAgent):
    def __init__(self):
        super().__init__()

        self.get_logger().info(f"My origin: {self.agentOrigins[self.id]}, All origins: {self.agentOrigins}")
        self.get_logger().info(f"My Voronoi cell: {self.graph.getVoronoiPartitions(self.agentOrigins)[self.agentOrigins[self.id]]}")


def main(args=None):
    # Initialize ROS2.
    rclpy.init(args=args)

    # Create the node and execute.
    agent = AhpaAgent()
    rclpy.spin(agent)

    # Destroy the node explicitly.
    agent.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
