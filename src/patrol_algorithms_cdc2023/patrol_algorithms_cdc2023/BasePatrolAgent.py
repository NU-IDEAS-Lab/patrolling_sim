import rclpy
from rclpy.node import Node

from patrol_algorithms_cdc2023.PatrolGraph import PatrolGraph

class BasePatrolAgent(Node):
    def __init__(self):
        super().__init__("PatrolAgent")
        print("Running!")
        self.graph = PatrolGraph("/home/anthony/idev/patrolling_sim/src/patrolling_sim/models/maps/cumberland/cumberland.graph")
        


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
