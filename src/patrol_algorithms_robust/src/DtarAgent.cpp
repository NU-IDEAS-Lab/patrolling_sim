#include "rclcpp/rclcpp.hpp"

#include "patrol_algorithms_robust/DtarAgent.hpp"
#include "patrol_algorithms_robust/GraphPartitioning.hpp"

int DtarAgent::compute_next_vertex() {
    // Random algorithm
    
    //number of neighbors of current vertex (number of existing possibilites)
    uint num_neighs = vertex_web[current_vertex].num_neigh;
    uint next_vertex;
    
    srand ( time(NULL) );
    int i = rand() % num_neighs;
    next_vertex = vertex_web[current_vertex].id_neigh[i];
    
    RCLCPP_INFO(this->get_logger(), "Random choice: %d", next_vertex);

    RCLCPP_WARN(this->get_logger(), "Cost of moving to next vertex: %u", graphGetEdgeCost(&vertex_web[current_vertex], &vertex_web[next_vertex]));
    
    return next_vertex;    
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DtarAgent>());
    rclcpp::shutdown();
    return 0; 
}
