/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <string>
#include <vector>
// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Int16MultiArray.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include "patrol_algorithms_base/getgraph.h"
#include "patrolling_sim_interfaces/message_types.h"

#define NUM_MAX_ROBOTS 32
#define INTERFERENCE_DISTANCE 2


typedef unsigned int uint;

class PatrolAgent : public rclcpp::Node {

protected:
    
    int agent_count;
    int ID_ROBOT;
    std::string tf_prefix;

    double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
    double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};

    rclcpp::TimerBase::SharedPtr timer;

    std::string graph_file, mapname;
    uint dimension; // Graph Dimension
    uint current_vertex; // current vertex
    bool ResendGoal; // Send the same goal again (if goal failed...)
    bool interference;
    double last_interference;
    bool goal_complete;
    bool initialize = true;
    bool end_simulation = false;
    int next_vertex = -1;
    // uint backUpCounter;
    vertex *vertex_web;
    double *instantaneous_idleness;  // local idleness
    double *last_visit;
    std::vector<int> vresults; // results exchanged among robots
    bool goal_canceled_by_user;
    double goal_reached_wait, communication_delay, last_communication_delay_time, lost_message_rate;
    std::string initial_positions;
    int aborted_count, resend_goal_count;
    
    using ActionNav2Pose = nav2_msgs::action::NavigateToPose;
    using ActionGoalHandleNav2Pose = rclcpp_action::ClientGoalHandle<ActionNav2Pose>;
    rclcpp_action::Client<ActionNav2Pose>::SharedPtr ac;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr positions_sub;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr results_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr positions_pub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr results_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clientCostmapLocalClear;

    
public:
    
    // PatrolAgent() { 
    //     listener=NULL;
    //     next_vertex = -1;
    //     initialize = true;
    //     end_simulation = false;
    //     ac = NULL;
    // }
    
    PatrolAgent();
    void ready();
    void initialize_node();
    void update_idleness();  // local idleness
    
    virtual void run_once();
    
    void getRobotPose(int robotid, float &x, float &y, float &theta);
    void odomCB(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    
    void sendGoal(int next_vertex);
    void cancelGoal();
    
    // void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalDoneCallback(const ActionGoalHandleNav2Pose::WrappedResult & result);
    // void goalActiveCallback();
    void goalActiveCallback(ActionGoalHandleNav2Pose::ConstSharedPtr goal_handle);
    // void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    void goalFeedbackCallback(ActionGoalHandleNav2Pose::ConstSharedPtr, const std::shared_ptr<const ActionNav2Pose::Feedback> feedback);

    
    void send_goal_reached();
    bool check_interference (int ID_ROBOT);
    void do_interference_behavior();
    void backup();
    void clearLocalCostmap(bool waitForService);
    
    void onGoalNotComplete(); // what to do when a goal has NOT been reached (aborted)
    
    // Events
    virtual void onGoalComplete(); // what to do when a goal has been reached
    virtual void processEvents();  // processes algorithm-specific events
    
    // Robot-Robot Communication
    void send_positions();
    void receive_positions();
    virtual void send_results();  // when goal is completed
    virtual void receive_results();  // asynchronous call
    void do_send_message(std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void send_interference();
    void positionsCB(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void resultsCB(std_msgs::msg::Int16MultiArray::ConstSharedPtr msg);
    
    // Must be implemented by sub-classes
    virtual int compute_next_vertex() = 0;

};


