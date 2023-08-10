import networkx as nx
import rclpy

from patrol_algorithms_cdc2023.BasePatrolAgent import BasePatrolAgent


class AhpaAgent(BasePatrolAgent):
    def __init__(self):
        super().__init__()

        self.get_logger().info(f"AHPA agent {self.id} has origin {self.agentOrigins[self.id]}.")

        # Set the allocation.
        self.voronoiOrigins = self.agentOrigins.copy()
        cell = self.getNodeAllocation(self.voronoiOrigins, self.agentOrigins)
        self.nodes = self.getNodeOrder(cell)
        self.currentNodeIdx = 1

        self.get_logger().info(f"Patrol order: {self.nodes}")

    def getNodeAllocation(self, origins, originalOrigins):
        ''' Returns the Voronoi partitions based on the origins provided. '''

        cells = nx.algorithms.voronoi.voronoi_cells(self.graph.graph, origins)
        return cells[originalOrigins[self.id]]

    def getNodeOrder(self, nodes):
        ''' Returns the visitation order for the provided nodes. '''

        if len(nodes) <= 1:
            return list(nodes)

        return nx.algorithms.approximation.traveling_salesman_problem(
            self.graph.graph,
            nodes=nodes,
            method=nx.algorithms.approximation.christofides
        )
    
    def getNextNode(self):
        ''' Returns the next node to visit. '''

        if self.currentNodeIdx >= len(self.nodes) - 1:
            self.currentNodeIdx = 0
        node = self.nodes[self.currentNodeIdx]
        self.currentNodeIdx += 1
        return node
    
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
