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
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <string>
// #include <ros/ros.h>
// #include <ros/package.h> //to get pkg path
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// // #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <nav_msgs/Odometry.h>
// #include <std_srvs/Empty.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

#include "patrol_algorithms_base/PatrolAgent.h"

using namespace std;

#define DELTA_TIME_SEQUENTIAL_START 15
#define SIMULATE_FOREVER true //WARNING: Set this to false, if you want a finishing condition.

// const std::string PS_path = ament_index_cpp::get_package_share_directory("patrolling_sim"); 	//D.Portugal => get pkg path

PatrolAgent::PatrolAgent() : rclcpp::Node("patrol_agent")
{
        /*
            argv[0]=/.../patrolling_sim/bin/GBS
            argv[1]=__name:=XXXXXX
            argv[2]=grid
            argv[3]=ID_ROBOT
        */
    
    srand ( time(NULL) );

    auto param_desc_id_robot = rcl_interfaces::msg::ParameterDescriptor{};
    // param_desc_id_robot.integer_range = {0, NUM_MAX_ROBOTS, 1}; //TODO
    this->declare_parameter("id_robot", 0, param_desc_id_robot);
    this->declare_parameter("patrol_graph_file", "");
    this->declare_parameter("initial_pos.x", 0.0);
    this->declare_parameter("initial_pos.y", 0.0);
    this->declare_parameter("goal_reached_wait", 3.0);
    this->declare_parameter("communication_delay", 0.2);
    this->declare_parameter("lost_message_rate", 0.0);
    this->declare_parameter("agent_count", 1);
    this->declare_parameter("tf_prefix", "");

    
    /** D.Portugal: needed in case you "rosrun" from another folder **/     
    // chdir(PS_path.c_str());

    this->goal_reached_wait = this->get_parameter("goal_reached_wait").get_parameter_value().get<double>();
    this->communication_delay = this->get_parameter("communication_delay").get_parameter_value().get<double>();
    this->lost_message_rate = this->get_parameter("lost_message_rate").get_parameter_value().get<double>();
    this->agent_count = this->get_parameter("agent_count").get_parameter_value().get<int>(); //TODO: Make this /control/agent_count
    this->tf_prefix = this->get_parameter("tf_prefix").get_parameter_value().get<std::string>();


    this->ID_ROBOT = this->get_parameter("id_robot").get_parameter_value().get<int>();
    
    RCLCPP_INFO(this->get_logger(), "Starting patrol agent %d", this->ID_ROBOT);

    // mapname = string(argv[2]);
    // graph_file = "maps/"+mapname+"/"+mapname+".graph";
    this->graph_file = this->get_parameter("patrol_graph_file").get_parameter_value().get<std::string>();  //TODO: Make this /control/patrol_graph_file
    
    //Check Graph Dimension:
    this->dimension = GetGraphDimension(graph_file.c_str());
    
    //Create Structure to save the Graph Info;
    this->vertex_web = new vertex[dimension];
    
    //Get the Graph info from the Graph File
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());
    
    
    uint nedges = GetNumberEdges(vertex_web,dimension);
    
    printf("Loaded graph %s with %d nodes and %d edges\n",graph_file.c_str(),dimension,nedges);

#if 0
    /* Output Graph Data */   
    for (i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif
    
    this->behavior_tree = ament_index_cpp::get_package_share_directory("nav2_bt_navigator") + "/behavior_trees/navigate_to_pose_w_replanning_goal_patience_and_recovery.xml";

    this->interference = false;
    this->ResendGoal = false;
    this->goal_complete = true;
    this->last_interference = 0;
    this->goal_canceled_by_user = false;
    this->aborted_count = 0;
    this->resend_goal_count = 0;
    this->communication_delay = 0.0;
    this->lost_message_rate = 0.0;
    this->goal_reached_wait = 0.0;
    /* Define Starting Vertex/Position (Launch File Parameters) */

    // // wait a random time (avoid conflicts with other robots starting at the same time...)
    // double r = 3.0 * ((rand() % 1000)/1000.0);
    // rclcpp::Duration wait(r); // seconds
    // wait.sleep();
    
    // double initial_x, initial_y;
    // std::vector<double> list;
    // nh.getParam("initial_pos", list);
    
    // if (list.empty()){
    //  RCLCPP_ERROR(this->get_logger(), "No initial positions given: check \"initial_pos\" parameter.");
    //  ros::shutdown();
    //  exit(-1);
    // }
       
    // int value = ID_ROBOT;
    // if (value == -1){value = 0;}
    
    // initial_x = list[2*value];
    // initial_y = list[2*value+1];

    double initial_x = this->get_parameter("initial_pos.x").get_parameter_value().get<double>();
    double initial_y = this->get_parameter("initial_pos.y").get_parameter_value().get<double>();
    
    //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
    this->current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    //   printf("initial vertex = %d\n\n",current_vertex);  
    
    
    //instantaneous idleness and last visit initialized with zeros:
    this->instantaneous_idleness = new double[dimension];
    this->last_visit = new double[dimension];
    for(size_t i=0; i<dimension; i++){ 
        instantaneous_idleness[i]= 0.0; 
        last_visit[i]= 0.0; 
        
        if (i==current_vertex){
            last_visit[i]= 0.1; //Avoids getting back at the initial vertex
        }
        //RCLCPP_INFO(this->get_logger(), "last_visit[%d]=%f", i, last_visit[i]);
    }
        
    //Publicar dados de "odom" para nó de posições
    // positions_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); //only concerned about the most recent
    this->positions_pub = this->create_publisher<nav_msgs::msg::Odometry>("/positions", 10);
        
    //Subscrever posições de outros robots
    // positions_sub = nh.subscribe<nav_msgs::Odometry>("positions", 10, boost::bind(&PatrolAgent::positionsCB, this, _1));  
    this->positions_sub = this->create_subscription<nav_msgs::msg::Odometry>("/positions", 100,
        std::bind(&PatrolAgent::positionsCB, this, std::placeholders::_1));
    
    char string1[40];
    char string2[40];
    
    // if(ID_ROBOT==-1){ 
    //     strcpy (string1,"odom"); //string = "odom"
    //     strcpy (string2,"cmd_vel"); //string = "cmd_vel"
    //     this->agent_count = 1;
    // }else{ 
    //     sprintf(string1,"robot_%d/odom",ID_ROBOT);
    //     sprintf(string2,"robot_%d/cmd_vel",ID_ROBOT);
    //     this->agent_count = ID_ROBOT + 1;
    // }   

    /* Set up listener for global coordinates of robots */
    this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->listener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

    //Cmd_vel to backup:
    // cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(string2, 1);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    //Subscrever para obter dados de "odom" do robot corrente
    // odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&PatrolAgent::odomCB, this, _1)); //size of the buffer = 1 (?)
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        std::bind(&PatrolAgent::odomCB, this, std::placeholders::_1));
    
    // Service client to clear the costmap.
    this->clientCostmapLocalClear = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "local_costmap/clear_entirely_local_costmap"
    );
    
    //Publicar dados para "results"
    results_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("/results", 100);
    results_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("/results", 100,
        std::bind(&PatrolAgent::resultsCB, this, std::placeholders::_1));

    // last time comm delay has been applied
    last_communication_delay_time = this->get_clock()->now().seconds();   

    // Wait for ready.
    ready();

    // Clear costmap initially.
    this->clearLocalCostmap(true);

    rclcpp::Rate rateRunLoop = rclcpp::Rate(30.0);

    timer = this->create_wall_timer(rateRunLoop.period(), std::bind(&PatrolAgent::run_once, this));
}

void PatrolAgent::ready() {
    
    // char move_string[40];
    
    // /* Define Goal */
    // if(ID_ROBOT==-1){ 
    //     strcpy (move_string,"move_base"); //string = "move_base
    // }else{
    //     sprintf(move_string,"robot_%d/move_base",ID_ROBOT);
    // }
    
    // ac = new MoveBaseClient("move_string", true);
    this->ac = rclcpp_action::create_client<ActionNav2Pose>(this, "navigate_to_pose");
    
    //wait for the action server to come up
    while(!this->ac->wait_for_action_server(1s)){
        if (!rclcpp::ok())
        {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for the nav2 action server to come up");
    } 
    RCLCPP_INFO(this->get_logger(), "Connected with nav2 action server");    
    
    rclcpp::Rate loop_rate(1); //1 sec

    // Wait for transform to become available.
    while(!this->tfBuffer->canTransform(this->tf_prefix + "base_link", this->tf_prefix + "map", tf2::TimePointZero))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform from map->base_link.");
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    } 
    RCLCPP_INFO(this->get_logger(), "Transform from map->base_link received.");

    
    /* Wait until all nodes are ready.. */
    while(initialize && rclcpp::ok()){
        initialize_node(); //announce that agent is alive
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }    

}

void PatrolAgent::run_once() {
    /* Run Algorithm */     
    if (goal_complete) {
        onGoalComplete();  // can be redefined
        resend_goal_count=0;
    }
    else { // goal not complete (active)
        if (interference) {
            do_interference_behavior();
        }       
        
        if (ResendGoal) {
            //Send the goal to the robot (Global Map)
            if (resend_goal_count<3) {
                resend_goal_count++;
                RCLCPP_INFO(this->get_logger(), "Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
                sendGoal(next_vertex);
            }
            else {
                resend_goal_count=0;
                onGoalNotComplete();
            }
            ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
        }
        
        processEvents();
        
        if (end_simulation) {
            return;
        }   
    
    } // if (goal_complete) 
}

void PatrolAgent::clearLocalCostmap(bool waitForService)
{
    if(waitForService)
    {
        while (!this->clientCostmapLocalClear->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the costmap clearing service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for costmap clearing service...");
        }
    }
    
    // nav2_msgs::srv::ClearEntireCostmap::Request request;
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto result = this->clientCostmapLocalClear->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Costmap correctly cleared before patrolling task.");
    }else{
        RCLCPP_WARN(this->get_logger(), "Was not able to clear costmap before patrolling...");
    }
}

void PatrolAgent::onGoalComplete()
{
    if(next_vertex>-1) {
        //Update Idleness Table:
        update_idleness();
        current_vertex = next_vertex;       
    }
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    /** SEND GOAL (REACHED) AND INTENTION **/
    send_goal_reached(); // Send TARGET to monitor
    send_results();  // Algorithm specific function

    //Send the goal to the robot (Global Map)
    RCLCPP_INFO(this->get_logger(), "Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base
    
    goal_complete = false;    
}

void PatrolAgent::onGoalNotComplete()
{   
    int prev_vertex = next_vertex;
    
    RCLCPP_INFO(this->get_logger(), "Goal not complete - From vertex %d to vertex %d\n", current_vertex, next_vertex);   
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    // Look for a random adjacent vertex different from the previous one
    int random_cnt=0;
    while (next_vertex == prev_vertex && random_cnt++<10) {
        int num_neighs = vertex_web[current_vertex].num_neigh;
        int i = rand() % num_neighs;
        next_vertex = vertex_web[current_vertex].id_neigh[i];
        RCLCPP_INFO(this->get_logger(), "Choosing another random vertex %d\n", next_vertex);
    }
    
    // Look for any random vertex different from the previous one
    while (next_vertex == prev_vertex && next_vertex == current_vertex) {
        int i = rand() % dimension;
        next_vertex = i;
        RCLCPP_INFO(this->get_logger(), "Choosing another random vertex %d\n", next_vertex);
    }

    //Send the goal to the robot (Global Map)
    RCLCPP_INFO(this->get_logger(), "Re-Sending NEW goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base
    
    goal_complete = false;    
}


void PatrolAgent::processEvents() {
    
}

void PatrolAgent::update_idleness() {
    double now = this->get_clock()->now().seconds();
        
    for(size_t i=0; i<dimension; i++){
        if ((int)i == next_vertex){
            last_visit[i] = now;    
        }
        instantaneous_idleness[i] = now - last_visit[i];           
	
	//Show Idleness Table:
	//RCLCPP_INFO(this->get_logger(), "idleness[%u] = %f",i,instantaneous_idleness[i]);
    }
}

void PatrolAgent::initialize_node (){ //ID,msg_type,1
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    RCLCPP_INFO(this->get_logger(), "Initialize Node: Robot %d",value); 
    
    std_msgs::msg::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(INITIALIZE_MSG_TYPE);
    msg.data.push_back(1);  // Robot initialized
    
    int count = 0;
    
    results_pub->publish(msg);

    /*
    //ATENÇÃO ao PUBLICADOR!
    rclcpp::Rate loop_rate(0.5); //meio segundo
    
    while (count<3){ //send activation msg 3times
        results_pub->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "publiquei msg: %s\n", msg.data.c_str());
        // ros::spinOnce();
        loop_rate.sleep();
        count++;
    }*/
}

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta) {
    
    if (listener==NULL) {
        RCLCPP_ERROR(this->get_logger(), "TF listener null");
        return;
    }
    
    std::string sframe = this->tf_prefix + "map";                //Patch David Portugal: Remember that the global map frame is "/map"
    std::string dframe = this->tf_prefix + "base_link";
    
    // tf::StampedTransform transform;
    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = this->tfBuffer->lookupTransform(
            dframe,
            sframe,
            tf2::TimePointZero,
            1s
        );
    }
    catch(const tf2::TransformException & ex)
    {
          RCLCPP_ERROR(
            this->get_logger(), "Could not transform %s to %s: %s",
            dframe.c_str(), sframe.c_str(), ex.what());
    }

    // x = transform.getOrigin().x();
    // y = transform.getOrigin().y();
    // theta = tf::getYaw(transform.getRotation());
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    
    // TODO: Fix this! TODOGOECKNER
    // theta = tf2::getYaw(transform.transform.rotation);
    theta = 0.0;

    // printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void PatrolAgent::odomCB(nav_msgs::msg::Odometry::ConstSharedPtr msg) { //colocar propria posicao na tabela
    
//  printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
    int idx = ID_ROBOT;
    
    if (ID_ROBOT<=-1){
        idx = 0;
    }

    // We cannot yet set the transform. Fail silently (other msgs will notify user).
    if(!this->tfBuffer->canTransform(this->tf_prefix + "base_link", this->tf_prefix + "map", tf2::TimePointZero))
    {
        return;
    }
    
    float x,y,th;
    getRobotPose(idx,x,y,th);
    
    xPos[idx]=x; // msg->pose.pose.position.x;
    yPos[idx]=y; // msg->pose.pose.position.y;
    
//  printf("Posicao colocada em Pos[%d]\n",idx);
}

void PatrolAgent::sendGoal(int next_vertex) 
{
    goal_canceled_by_user = false;
    
    double target_x = vertex_web[next_vertex].x, 
           target_y = vertex_web[next_vertex].y;
    
    //Define Goal:
    // move_base_msgs::MoveBaseGoal goal;
    // //Send the goal to the robot (Global Map)
    // geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);     
    // goal.target_pose.header.frame_id = "map"; 
    // goal.target_pose.header.stamp = this->get_clock()->now();    
    // goal.target_pose.pose.position.x = target_x; // vertex_web[current_vertex].x;
    // goal.target_pose.pose.position.y = target_y; // vertex_web[current_vertex].y;  
    // goal.target_pose.pose.orientation = angle_quat; //doesn't matter really.
    // ac->sendGoal(goal, boost::bind(&PatrolAgent::goalDoneCallback, this, _1, _2), boost::bind(&PatrolAgent::goalActiveCallback,this), boost::bind(&PatrolAgent::goalFeedbackCallback, this,_1));  

    auto goal = ActionNav2Pose::Goal();
    goal.behavior_tree = this->behavior_tree;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->get_clock()->now();    
    goal.pose.pose.position.x = target_x; // vertex_web[current_vertex].x;
    goal.pose.pose.position.y = target_y; // vertex_web[current_vertex].y;  
    goal.pose.pose.orientation.x = 0;
    goal.pose.pose.orientation.y = 0;
    goal.pose.pose.orientation.z = 0;
    goal.pose.pose.orientation.w = 1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ActionNav2Pose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PatrolAgent::goalActiveCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&PatrolAgent::goalFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&PatrolAgent::goalDoneCallback, this, std::placeholders::_1);
    this->ac->async_send_goal(goal, send_goal_options);
}

void PatrolAgent::cancelGoal() 
{
    goal_canceled_by_user = true;
    ac->async_cancel_all_goals();
}

void PatrolAgent::goalDoneCallback(const ActionGoalHandleNav2Pose::WrappedResult & result){ //goal terminado (completo ou cancelado)
//  RCLCPP_INFO(this->get_logger(), "Goal is complete (suceeded, aborted or cancelled).");
    // If the goal succeeded send a new one!
    //if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
    // If it was aborted time to back up!
    //if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;    
    
    if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(this->get_logger(), "Goal reached ... WAITING %.2f sec",goal_reached_wait);
        rclcpp::Duration delay = rclcpp::Duration::from_seconds(this->goal_reached_wait); // wait after goal is reached
        this->get_clock()->sleep_for(delay);
        RCLCPP_INFO(this->get_logger(), "Goal reached ... DONE");
        goal_complete = true;
    }else{
        aborted_count++;
        RCLCPP_INFO(this->get_logger(), "CANCELLED or ABORTED... %d",aborted_count);   //tentar voltar a enviar goal..
        if (!goal_canceled_by_user) {
            RCLCPP_INFO(this->get_logger(), "Goal not cancelled by the interference...");

            //RCLCPP_INFO(this->get_logger(), "Backup");
            // backup();

            // RCLCPP_INFO(this->get_logger(), "Clear costmap!");
            // this->clearLocalCostmap(false);

            RCLCPP_INFO(this->get_logger(), "Resend Goal!");
            ResendGoal = true;
        }
    }
}

void PatrolAgent::goalActiveCallback(ActionGoalHandleNav2Pose::ConstSharedPtr goal_handle){  //enquanto o robot esta a andar para o goal...
    goal_complete = false;
//      RCLCPP_INFO(this->get_logger(), "Goal is active.");
}

void PatrolAgent::goalFeedbackCallback(ActionGoalHandleNav2Pose::ConstSharedPtr, const std::shared_ptr<const ActionNav2Pose::Feedback> feedback){    //publicar posições

    send_positions();
    
    int value = ID_ROBOT;
    if (value==-1){ value = 0;}
    interference = check_interference(value);    
}

void PatrolAgent::send_goal_reached() {
    
    int value = ID_ROBOT;
    if (value==-1){ value = 0;}
    
    // [ID,msg_type,vertex,intention,0]
    std_msgs::msg::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(TARGET_REACHED_MSG_TYPE);
    msg.data.push_back(current_vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    
    results_pub->publish(msg);   
}

bool PatrolAgent::check_interference (int robot_id){ //verificar se os robots estao proximos
    
    return false;


    int i;
    double dist_quad;
    
    if (this->get_clock()->now().seconds()-last_interference<10)  // seconds
        return false; // false if within 10 seconds from the last one
    
    /* Poderei usar this->agent_count para afinar */
    for (i=0; i<robot_id; i++){ //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
        
        dist_quad = (xPos[i] - xPos[robot_id])*(xPos[i] - xPos[robot_id]) + (yPos[i] - yPos[robot_id])*(yPos[i] - yPos[robot_id]);
        
        if (dist_quad <= INTERFERENCE_DISTANCE*INTERFERENCE_DISTANCE){    //robots are ... meter or less apart
//          RCLCPP_INFO(this->get_logger(), "Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            last_interference = this->get_clock()->now().seconds();
            return true;
        }       
    }
    return false;
    
}

void PatrolAgent::backup(){
    
    rclcpp::Rate loop_rate(100); // 100Hz
    
    int backUpCounter = 0;
    while (backUpCounter<=100){
    
      if(backUpCounter==0){
          RCLCPP_INFO(this->get_logger(), "The wall is too close! I need to do some backing up...");
          // Move the robot back...
          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = -0.1;
          cmd_vel.angular.z = 0.0;
          cmd_vel_pub->publish(cmd_vel);
      }
              
      if(backUpCounter==20){
          // Turn the robot around...
          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.5;
          cmd_vel_pub->publish(cmd_vel);
      }
              
      if(backUpCounter==100){
          // Stop the robot...
          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          cmd_vel_pub->publish(cmd_vel);
              
          // RCLCPP_INFO(this->get_logger(), "Done backing up, now on with my life!");      
      }

    //   ros::spinOnce();
      loop_rate.sleep();
      backUpCounter++;
    
    }
    
}

void PatrolAgent::do_interference_behavior()
{
    RCLCPP_INFO(this->get_logger(), "Interference detected! Executing interference behavior...\n");   
    send_interference();  // send interference to monitor for counting
    
#if 1
    // Stop the robot..         
    cancelGoal();
    RCLCPP_INFO(this->get_logger(), "Robot stopped");
    rclcpp::Duration delay = rclcpp::Duration::from_seconds(3);
    this->get_clock()->sleep_for(delay);
    ResendGoal = true;
#else    
    //get own "odom" positions...
    ros::spinOnce();        
                
    //Waiting until conflict is solved...
    int value = ID_ROBOT;
    if (value == -1){ value = 0;}
    while(interference){
        interference = check_interference(value);
        if (goal_complete || ResendGoal){
            interference = false;
        }
    }
#endif
}




// ROBOT-ROBOT COMMUNICATION



void PatrolAgent::send_positions()
{
    //Publish Position to common node:
    nav_msgs::msg::Odometry msg; 
    
    int idx = ID_ROBOT;

    if (ID_ROBOT <= -1){
        msg.header.frame_id = "map";    //identificador do robot q publicou
        idx = 0;
    }else{
        char string[20];
        sprintf(string,"robot_%d/map",ID_ROBOT);
        msg.header.frame_id = string;
    }

    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    msg.pose.pose.position.y = yPos[idx]; //send odometry.y
  
    positions_pub->publish(msg);
    // ros::spinOnce();
}


void PatrolAgent::receive_positions()
{
    
}

void PatrolAgent::positionsCB(nav_msgs::msg::Odometry::ConstSharedPtr msg) { //construir tabelas de posições
        
//     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);    
        
    char id[20]; //identificador do robot q enviou a msg d posição...
    strcpy( id, msg->header.frame_id.c_str() );
    //int stamp = msg->header.seq;
//     printf("robot q mandou msg = %s\n", id);
    
    // Build Positions Table
    
    if (ID_ROBOT>-1){
    //verify id "XX" of robot: (string: "robot_XX/map")
    
        char str_idx[4];
        uint i;
        
        for (i=6; i<10; i++){
            if (id[i]=='/'){
                str_idx[i-6] = '\0';
                break;
            }else{
                str_idx[i-6] = id[i];
            }
        }
        
        int idx = atoi (str_idx);
    //  printf("id robot q mandou msg = %d\n",idx);
        
        // if (idx >= this->agent_count && this->agent_count <= NUM_MAX_ROBOTS){
        //     //update this->agent_count:
        //     this->agent_count = idx+1;
        // }
        
        if (ID_ROBOT != idx){  //Ignore own positions   
            xPos[idx]=msg->pose.pose.position.x;
            yPos[idx]=msg->pose.pose.position.y;        
        }   
//      printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos[idx], idx, yPos[idx] );       
    }
    
    receive_positions();
}

void PatrolAgent::send_results() { 

}

// simulates blocking send operation with delay in communication
void PatrolAgent::do_send_message(std_msgs::msg::Int16MultiArray::SharedPtr msg) {
	if (communication_delay>0.001) {
    	//double current_time = this->get_clock()->now().seconds();
    	//if (current_time-last_communication_delay_time>1.0) { 
	        //RCLCPP_INFO(this->get_logger(), "Communication delay %.1f",communication_delay);
            rclcpp::Duration delay = rclcpp::Duration::from_seconds(communication_delay);
            this->get_clock()->sleep_for(delay);
	        //last_communication_delay_time = current_time;
        //}
    }    
    results_pub->publish(*msg);
    // ros::spinOnce();
}


void PatrolAgent::receive_results() {

}

void PatrolAgent::send_interference(){
    //interference: [ID,msg_type]
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    printf("Send Interference: Robot %d\n",value);   
    
    std_msgs::msg::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(INTERFERENCE_MSG_TYPE);
    
    results_pub->publish(msg);   
    // ros::spinOnce();
}




void PatrolAgent::resultsCB(std_msgs::msg::Int16MultiArray::ConstSharedPtr msg) { 
    
    std::vector<signed short>::const_iterator it = msg->data.begin();    
    
    vresults.clear();
    
    for (size_t k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    } 

    int id_sender = vresults[0];
    int msg_type = vresults[1];
    
    // RCLCPP_INFO(this->get_logger(), " MESSAGE FROM %d TYPE %d ...",id_sender, msg_type);
    
    // messages coming from the monitor
    if (id_sender==-1 && msg_type==INITIALIZE_MSG_TYPE) {
        if (initialize==true && vresults[2]==100) {   //"-1,msg_type,100,seq_flag" (BEGINNING)
            RCLCPP_INFO(this->get_logger(), "Let's Patrol!\n");
            double r = 1.0 * ((rand() % 1000)/1000.0);

            //TODO if sequential start
            //r = DELTA_TIME_SEQUENTIAL_START * ID_ROBOT;

            rclcpp::Duration wait = rclcpp::Duration::from_seconds(r); // seconds

            // printf("Wait %.1f seconds (init pos:%s)\n",r,initial_positions.c_str());

            this->get_clock()->sleep_for(wait);
            initialize = false;
        }

#if SIMULATE_FOREVER == false
        if (initialize==false && vresults[2]==999) {   //"-1,msg_type,999" (END)
            RCLCPP_INFO(this->get_logger(), "The simulation is over. Let's leave");
            end_simulation = true;     
        }
#endif        
    }
    
    if (!initialize) {
        if(ID_ROBOT>-1){
#if 0
            // communication delay
            if ((communication_delay>0.001) && (id_sender!=ID_ROBOT)) {
                    double current_time = this->get_clock()->now().seconds();
                    if (current_time-last_communication_delay_time>1.0) { 
                            RCLCPP_INFO(this->get_logger(), "Communication delay %.1f",communication_delay);
                            rclcpp::Duration delay(communication_delay); // seconds
                            delay.sleep();
                            last_communication_delay_time = current_time;
                }
            }
#endif
            bool lost_message = false;
            if ((lost_message_rate>0.0001)&& (id_sender!=ID_ROBOT)) {
                double r = (rand() % 1000)/1000.0;
                lost_message = r < lost_message_rate;
            }
            if (lost_message) {
                RCLCPP_INFO(this->get_logger(), "Lost message");
            }
        }
        receive_results();
    }

    // ros::spinOnce();
  
}