import rclpy

from std_msgs.msg import Int16MultiArray

from patrol_algorithms_ahpa.BasePatrolAgent import BasePatrolAgent
from patrolling_sim_interfaces.srv import QueryPolicy


class PzAgent(BasePatrolAgent):
    def __init__(self):
        self.pzReady = False
        super().__init__()

        # Create the service client to query policy.
        # The policy could simply be loaded here, but for the sake of time we just query the policy from the policy server.
        self.policyClient = self.create_client(QueryPolicy, "/pz_query_policy")

        # Wait for the service to be ready.
        while not self.policyClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for policy service...")
        
        self.get_logger().info(f"PZ agent {self.id} initialization complete.")
        self.pzReady = True

    def onExperimentInitialized(self):
        ''' Begins execution after experiment initialization completes. '''

        self.experimentInitialized = True
        self.timeStart = self.get_clock().now().to_msg().sec
        self.get_logger().info("Let's Patrol!")

        self.getNextNode()

    def onNavigationGoalSuccess(self):
        ''' Called when we have successfully navigated to a node. '''

        # Notify the monitor.
        msg = Int16MultiArray()
        msg.data = [self.id, self.MSG_TYPES["TARGET_REACHED_MSG_TYPE"], self.goalNode]
        self.pubResults.publish(msg)

        # Get next node.
        self.getNextNode()
    
    def getNextNode(self):
        ''' Returns the next node to visit. '''

        self.get_logger().info(f"PZ agent {self.id} querying policy.")

        # Set up a query.
        request = QueryPolicy.Request()
        request.agent_id = self.id

        # Query the policy.
        future = self.policyClient.call_async(request)
        future.add_done_callback(self.onPolicyQuerySuccess)
        self.get_logger().info(f"PZ agent {self.id} sent policy query.")
    
    def onPolicyQuerySuccess(self, future):
        ''' Called when we have successfully queried the policy. '''

        if future.result() is not None:
            nextNode = future.result().node_id
        else:
            self.get_logger().error(f"Failed to query policy for agent {self.id}.")
            nextNode = None

        self.goToNode(nextNode)


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