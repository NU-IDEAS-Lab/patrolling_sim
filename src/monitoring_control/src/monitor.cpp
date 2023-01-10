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

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include <string>
#include <climits>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;

#include "patrolling_sim/getgraph.h"
#include "patrolling_sim_interfaces/message_types.h"
#include "patrolling_sim_interfaces/srv/go_to_start_pos_srv.hpp"

#define NUM_MAX_ROBOTS 32
#define DEAD_ROBOT_TIME 300.0 // (seconds) time from last goal reached after which a robot is considered dead
#define TIMEOUT_WRITE_RESULTS 180.0 // (seconds) timeout for writing results to file 
// For hystograms
#define RESOLUTION 1.0 // seconds
#define MAXIDLENESS 500.0 // seconds

#define LOG_MONITOR 0
#define SAVE_HYSTOGRAMS 0
#define EXTENDED_STAGE 0

#define SIMULATE_FOREVER true                   //WARNING: Set this to false, if you want a finishing condition.
#define TIMEOUT_WRITE_RESULTS_FOREVER 900.0     // timeout for writing results to file when simulating forever

using std::string;
using std::cout;
using std::endl;

class PatrolMonitor : public rclcpp::Node
{
    public:
    
    typedef unsigned int uint;

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr  results_sub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr results_pub, screenshot_pub;

    //Initialization:
    bool initialize = true; // Initialization flag
    bool goto_start_pos = false; //default: robots already start in right position
    uint cnt=0;  // Count number of robots connected
    uint teamsize;
    bool init_robots[NUM_MAX_ROBOTS];
    double last_goal_reached[NUM_MAX_ROBOTS];

    // mutex for accessing last_goal_reached vector
    pthread_mutex_t lock_last_goal_reached;

    //State Variables:
    bool goal_reached = false;

    int goal;
    double time_zero, last_report_time;
    time_t real_time_zero;
    double goal_reached_wait,comm_delay,lost_message_rate;
    string algorithm, algparams, nav_mod, initial_positions;

    // const std::string PS_path = ros::package::getPath("patrolling_sim"); 	//D.Portugal => get pkg path

    #define MAX_DIMENSION 200

    /* ESTRUTURAS DE DADOS A CALCULAR */
    double last_visit [MAX_DIMENSION], current_idleness [MAX_DIMENSION], avg_idleness [MAX_DIMENSION], stddev_idleness [MAX_DIMENSION];
    double total_0 [MAX_DIMENSION], total_1 [MAX_DIMENSION],  total_2[MAX_DIMENSION];
    int number_of_visits [MAX_DIMENSION];
    size_t dimension; // graph size

    double worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, previous_avg_graph_idl = DBL_MAX;
    // global measures
    double min_idleness = 0.0, max_idleness = 0.0;
    double gavg, gstddev;
    double gT0=0.0, gT1=0.0, gT2=0.0;

    uint interference_cnt = 0;
    uint complete_patrol = 0;
    uint patrol_cnt = 1;

    #if SAVE_HYSTOGRAMS
    #define hn ((int)(MAXIDLENESS/RESOLUTION)+1)
    int hsum;
    int hv[hn];
    #endif

    // Idleness file
    FILE *idlfile;

    // log file
    FILE *logfile = NULL;

    FILE *resultstimecsvfile;
    string idlfilename,resultsfilename,resultstimecsvfilename,expname;

    rclcpp::TimerBase::SharedPtr timer;

    string graph_file;
    char teamsize_str[10];
    string mapname;
    char hostname[80];
    bool dead = false; // check if there is a dead robot
    bool simrun, simabort; // check if simulation is running and if it has been aborted by the user
    char strnow[80];

    PatrolMonitor() : rclcpp::Node("monitor")
    {
          /*
            argc=3
            argv[0]=/.../patrolling_sim/bin/monitor
            argv[1]=grid
            argv[2]=ALGORITHM = {MSP,Cyc,CC,CR,HCR}
            argv[3]=TEAMSIZE
            */

        this->declare_parameter("map", "cumberland");
        this->declare_parameter("algorithm_name", "Random");
        this->declare_parameter("patrol_graph_file", "");
        this->declare_parameter("goal_reached_wait", 0.0);
        this->declare_parameter("communication_delay", 0.0);
        this->declare_parameter("lost_message_rate", 0.0);
        this->declare_parameter("agent_count", 1);
        this->declare_parameter("simulation_running", false);
        this->declare_parameter("simulation_abort", false);
        this->declare_parameter("initial_positions", "default");
        this->declare_parameter("navigation_module", "ros");

        teamsize = this->get_parameter("agent_count").get_parameter_value().get<int>();
        
        if ( teamsize >= NUM_MAX_ROBOTS || teamsize <1 ){
            RCLCPP_INFO(this->get_logger(), "The Teamsize must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
            throw std::invalid_argument("The Teamsize must be an integer number between 1 and NUM_MAX_ROBOTS");
        }else{
            strcpy (teamsize_str, std::to_string(teamsize).c_str()); 
            //  RCLCPP_INFO(this->get_logger(), "teamsize: %s", teamsize_str);
            //  RCLCPP_INFO(this->get_logger(), "teamsize: %u", teamsize);
        }
        
        
        algorithm = this->get_parameter("algorithm_name").get_parameter_value().get<string>();
        RCLCPP_INFO(this->get_logger(), "Algorithm: %s\n",algorithm.c_str());
        
        mapname = this->get_parameter("map").get_parameter_value().get<string>();
        graph_file = this->get_parameter("patrol_graph_file").get_parameter_value().get<string>();

        RCLCPP_INFO(this->get_logger(), "Graph: %s\n",graph_file.c_str());
            
        /** D.Portugal: needed in case you "rosrun" from another folder **/     
        // chdir(PS_path.c_str());
        
        //Check Graph Dimension:
        dimension = GetGraphDimension(graph_file.c_str());
        if (dimension>MAX_DIMENSION) {
            cout << "ERROR!!! dimension > MAX_DIMENSION (static value) !!!" << endl;
            abort();
        }
        RCLCPP_INFO(this->get_logger(), "Dimension: %u\n",(uint)dimension);
            
        int r = gethostname(hostname,80);
        if (r<0)
            strcpy(hostname,"default");
            
        RCLCPP_INFO(this->get_logger(), "Host name: %s\n",hostname);
            
        for (size_t i=0; i<dimension; i++){
            number_of_visits[i] = -1;  // first visit should not be cnted for avg
            current_idleness[i] = 0.0;
            last_visit[i] = 0.0;
        }
        
        for (size_t i=0; i<NUM_MAX_ROBOTS; i++){
            init_robots[i] = false;
            last_goal_reached[i] = 0.0;
        }

        // Scenario name (to be used in file and directory names)
        char sname[80];
        scenario_name(sname,graph_file.c_str(),teamsize_str);
        

        // Create directory results if does not exist
        string path1 = "results";
        
        string path2,path3,path4;
        
        path2 = path1 + "/" + string(sname);
        path3 = path2 + "/" + algorithm;
        path4 = path3 + "/" + hostname;

        
        struct stat st;
        
        if (stat(path1.c_str(), &st) != 0)
        mkdir(path1.c_str(), 0777);
        if (stat(path2.c_str(), &st) != 0)
        mkdir(path2.c_str(), 0777);
        if (stat(path3.c_str(), &st) != 0)
        mkdir(path3.c_str(), 0777);
        if (stat(path4.c_str(), &st) != 0)
        mkdir(path4.c_str(), 0777);

        RCLCPP_INFO(this->get_logger(), "Path experimental results: %s\n",path4.c_str());
        
        // Local time (real clock time)
        time_t rawtime;
        struct tm * timeinfo;
        
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        sprintf(strnow,"%d%02d%02d_%02d%02d%02d",  timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
        RCLCPP_INFO(this->get_logger(), "Date-time of the experiment: %s\n",strnow);
        
        // File to log all the idlenesses of an experimental scenario

        expname = path4 + "/" + string(strnow);
        idlfilename = expname + "_idleness.csv";
        resultsfilename = expname + "_results.txt";
        resultstimecsvfilename = expname + "_timeresults.csv";

        FILE *fexplist;
        fexplist = fopen("experiments.txt", "a");
        fprintf(fexplist,"%s\n",expname.c_str());
        fclose(fexplist);

        idlfile = fopen (idlfilename.c_str(),"a");
        fprintf(idlfile,"Time;Robot;Node;Idleness;Interferences\n"); // header

        resultstimecsvfile = fopen(resultstimecsvfilename.c_str(), "w");

        fprintf(resultstimecsvfile,"Time;Idleness min;Idleness avg;Idleness stddev;Idleness max;Interferences\n"); // header

        #if LOG_MONITOR
            char logfilename[80];
            sprintf(logfilename,"monitor_%s.log",strnow);
            logfile = fopen(logfilename, "w");
        #endif

            dolog("Monitor node starting");
            dolog(expname.c_str());

        #if SAVE_HYSTOGRAMS    
            // Vectors for hystograms
            for (int k=0; k<hn; k++) hv[k]=0;
            hsum=0;
        #endif
                
        //Subscribe "results" from robots
        results_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>("/results", 100, std::bind(&PatrolMonitor::resultsCB, this, std::placeholders::_1)); 
        
        //Publish data to "results"
        results_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("/results", 100);
        
        #if EXTENDED_STAGE  
            screenshot_pub = nh.advertise<std_msgs::String>("/stageGUIRequest", 100);
        #endif    
        
        rclcpp::Rate loop_rate(1.0); // 1 Hz
        
        this->set_parameter(rclcpp::Parameter("simulation_running", true));
        this->set_parameter(rclcpp::Parameter("simulation_abort", false));
        
        // TODO GOECKNER - Temporarily disable /GoToStartPosSrv
        // if(ros::service::exists("/GotoStartPosSrv", false)){ //see if service has been advertised or not
        //     goto_start_pos = true;  //if service exists: robots need to be sent to starting positions
        //     RCLCPP_INFO(this->get_logger(), "/GotoStartPosSrv is advertised. Robots will be sent to starting positions.");
        // }else{
        //     RCLCPP_WARN(this->get_logger(), "/GotoStartPosSrv does not exist. Assuming robots are already at starting positions.");
        // }
                    
        // read parameters
        try
        {
            goal_reached_wait = this->get_parameter("goal_reached_wait").get_parameter_value().get<double>();
        }
        catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            goal_reached_wait = 0.0;
            RCLCPP_WARN(this->get_logger(), "Cannot read parameter /goal_reached_wait. Using default value 0.0!");
        }

        try
        {
            comm_delay = this->get_parameter("communication_delay").get_parameter_value().get<double>();
        }
        catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            comm_delay = 0.0;
            RCLCPP_WARN(this->get_logger(), "Cannot read parameter /communication_delay. Using default value 0.0!");
        }

        try
        {
            lost_message_rate = this->get_parameter("lost_message_rate").get_parameter_value().get<double>();
        }
        catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            lost_message_rate = 0.0;
            RCLCPP_WARN(this->get_logger(), "Cannot read parameter /lost_message_rate. Using default value 0.0!");
        }

        try
        {
            initial_positions = this->get_parameter("initial_positions").get_parameter_value().get<string>();
        }
        catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            initial_positions = "default";
            RCLCPP_WARN(this->get_logger(), "Cannot read parameter /initial_positions. Using default value 'default'!");
        }

        try
        {
            nav_mod = this->get_parameter("navigation_module").get_parameter_value().get<string>();
        }
        catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            nav_mod = "ros";
            RCLCPP_WARN(this->get_logger(), "Cannot read parameter /navigation_module. Using default value 'ros'!");
        }

        // mutex for accessing last_goal_reached vector
        pthread_mutex_init(&lock_last_goal_reached, NULL);

        timer = this->create_wall_timer(loop_rate.period(), std::bind(&PatrolMonitor::run_once, this));
    }


    void run_once()
    {
        dolog("main loop - begin");

        if (!initialize)
        {  //check if msg is goal or interference -> compute necessary results.
                
            // check time
            double report_time = this->get_clock()->now().seconds();
            
            // RCLCPP_INFO(this->get_logger(), "### report time=%.1f  last_report_time=%.1f diff = %.1f\n",report_time, last_report_time, report_time - last_report_time);
            
            // write results every TIMEOUT_WRITE_RESULTS_(FOREVER) seconds anyway
            bool timeout_write_results;
            
            #if SIMULATE_FOREVER
                timeout_write_results = (report_time - last_report_time > TIMEOUT_WRITE_RESULTS_FOREVER);
            #else
                timeout_write_results = (report_time - last_report_time > TIMEOUT_WRITE_RESULTS);
            #endif
            
            if ((patrol_cnt == complete_patrol) || timeout_write_results){ 

                dolog("main loop - write results begin");

                if (complete_patrol==1) {
                    try
                    {
                        algparams = this->get_parameter("algorithm_params").get_parameter_value().get<string>();
                    }
                    catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
                    {
                        RCLCPP_WARN(this->get_logger(), "Cannot read parameter /algorithm_params.");
                    }

                    try
                    {
                        goal_reached_wait = this->get_parameter("goal_reached_wait").get_parameter_value().get<double>();
                    }
                    catch(rclcpp::exceptions::ParameterNotDeclaredException &e)
                    {
                        goal_reached_wait = 0.0;
                        RCLCPP_WARN(this->get_logger(), "Cannot read parameter /goal_reached_wait. Using default value 0.0!");
                    }
                }

                // write results every time a patrolling cycle is finished.
                // or after some time
                previous_avg_graph_idl = avg_graph_idl; //save previous avg idleness graph value

                RCLCPP_INFO(this->get_logger(), "******************************************\n");
                RCLCPP_INFO(this->get_logger(), "Patrol completed [%d]. Write to File!\n",complete_patrol);

                worst_avg_idleness = 0.0;
                avg_graph_idl = 0.0;
                stddev_graph_idl = 0.0;
                avg_stddev_graph_idl = 0.0;
                        
                // Compute avg and stddev
                double T0=0.0,T1=0.0,T2=0.0,S1=0.0;
                for (size_t i=0; i<dimension; i++){
                    T0++; T1+=avg_idleness[i]; T2+=avg_idleness[i]*avg_idleness[i];
                    S1+=stddev_idleness[i];
                    if ( avg_idleness[i] > worst_avg_idleness ){
                        worst_avg_idleness = avg_idleness[i];
                    }
                }
                        
                avg_graph_idl = T1/T0;
                stddev_graph_idl = 1.0/T0 * sqrt(T0*T2-T1*T1);
                avg_stddev_graph_idl = S1/T0;
                // global stats
                gavg = gT1/gT0;
                gstddev = 1.0/gT0 * sqrt(gT0*gT2-gT1*gT1);

                uint tot_visits=0;
                for (size_t i=0; i<dimension; i++){
                    tot_visits += number_of_visits[i];
                }
                float avg_visits = (float)tot_visits/dimension;

                double duration = report_time-time_zero;
                time_t real_now; time (&real_now); 
                double real_duration = (double)real_now - (double)real_time_zero;				
                
                RCLCPP_INFO(this->get_logger(), "Node idleness\n");
                RCLCPP_INFO(this->get_logger(), "   worst_avg_idleness (graph) = %.2f\n", worst_avg_idleness);
                RCLCPP_INFO(this->get_logger(), "   avg_idleness (graph) = %.2f\n", avg_graph_idl);
                median_graph_idl = Median (avg_idleness, dimension);
                RCLCPP_INFO(this->get_logger(), "   median_idleness (graph) = %.2f\n", median_graph_idl);
                RCLCPP_INFO(this->get_logger(), "   stddev_idleness (graph) = %.2f\n", stddev_graph_idl);

                RCLCPP_INFO(this->get_logger(), "Global idleness\n");
                RCLCPP_INFO(this->get_logger(), "   min = %.1f\n", min_idleness);
                RCLCPP_INFO(this->get_logger(), "   avg = %.1f\n", gavg);
                RCLCPP_INFO(this->get_logger(), "   stddev = %.1f\n", gstddev);
                RCLCPP_INFO(this->get_logger(), "   max = %.1f\n", max_idleness);

                RCLCPP_INFO(this->get_logger(), "\nInterferences\t%u\nInterference rate\t%.2f\nVisits\t%u\nAvg visits per node\t%.1f\nTime Elapsed\t%.1f\nReal Time Elapsed\t%.1f\n",
                    interference_cnt,(float)interference_cnt/duration*60,tot_visits,avg_visits,duration,real_duration);
                
                if (timeout_write_results)
                last_report_time = report_time;
                else
                patrol_cnt++;
                
                        
                double tolerance = 0.025 * avg_graph_idl;  //2.5% tolerance
                printf ("diff avg_idleness = %.1f\n",fabs(previous_avg_graph_idl - avg_graph_idl));
                printf ("tolerance = %.1f\n",tolerance);
                
                // write results to file
                if (!timeout_write_results)
                    write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
                        worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
                        min_idleness, gavg, gstddev, max_idleness,
                        interference_cnt, tot_visits, avg_visits,
                        graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
                        resultsfilename);
                else {
                    /*
                    write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
                        worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl,
                        min_idleness, gavg, gstddev, max_idleness,
                        interference_cnt, tot_visits, avg_visits,
                        graph_file.c_str(), teamsize_str, duration, real_duration, comm_delay,
                        resultstimefilename);
                    */

                    fprintf(resultstimecsvfile,"%.1f;%.1f;%.1f;%.1f;%.1f;%d\n", 
                                duration,min_idleness,gavg,gstddev,max_idleness,interference_cnt);
                    fflush(resultstimecsvfile);

                }

                dolog("main loop - write results begin");

            } // if ((patrol_cnt == complete_patrol) || timeout_write_results)
    

            dolog("    check - begin");

            // Check if simulation must be terminated
            #if SIMULATE_FOREVER == false
                dead = check_dead_robots();
                            
                simrun=true; simabort=false;
                std::string psimrun, psimabort; bool bsimabort;
                if (nh.getParam("simulation_running", psimrun))
                    if (psimrun=="false")
                        simrun = false;
                if (nh.getParam("simulation_abort", psimabort))
                    if (psimabort=="true")
                        simabort = true;
                if (nh.getParam("simulation_abort", bsimabort))
                    simabort = bsimabort;
                    
                if ( (dead) || (!simrun) || (simabort) ) {
                    printf ("Simulation is Over\n");                
                    nh.setParam("simulation_running", false);
                    finish_simulation ();
                    ros::spinOnce();
                    break;
                }
            #endif

            dolog("    check - end");

        } // if ! initialize  
        
        dolog("main loop - end");
    }

    void on_experiment_end()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down experiment.");

        fclose(idlfile);
        fclose(resultstimecsvfile);
    

        double duration = this->get_clock()->now().seconds() - time_zero;
        time_t real_now; time (&real_now); 
        double real_duration = (double)real_now - (double)real_time_zero;

        uint tot_visits=0;
        for (size_t i=0; i<dimension; i++){
            tot_visits += number_of_visits[i];
        }
        float avg_visits = (float)tot_visits/dimension;


        // Write info file with overall results 
        string infofilename;
        infofilename = expname + "_info.csv";

        FILE *infofile;
        infofile = fopen (infofilename.c_str(),"w");
        fprintf(infofile,"%s;%s;%s;%.1f;%.2f;%s;%s;%s;%s;%s;%.1f;%.1f;%d;%s;%.1f;%.1f;%.1f;%.1f;%.2f;%d;%.1f;%d\n",
                mapname.c_str(),teamsize_str,initial_positions.c_str(),goal_reached_wait,comm_delay,nav_mod.c_str(),
                algorithm.c_str(), algparams.c_str(),hostname,
                strnow,duration,real_duration,interference_cnt,(dead?"FAIL":(simabort?"ABORT":"TIMEOUT")),
                min_idleness, gavg, gstddev, max_idleness, (float)interference_cnt/duration*60,
                tot_visits, avg_visits, complete_patrol
        );

        fclose(infofile);
        cout << "Info file " << infofilename << " saved." << endl;


        #if SAVE_HYSTOGRAMS
            // Hystogram files
            string hfilename,chfilename;
            hfilename = expname + ".hist";
            chfilename = expname + ".chist";

            cout << "Histogram output files: " << hfilename << endl;
            std::ofstream of1; of1.open(hfilename.c_str());
            std::ofstream of2; of2.open(chfilename.c_str());
            double c=0;
            for (int k=0; k<hn; k++) {
                of1 << k*RESOLUTION << " " << (double)hv[k]/hsum << endl;
                c += (double)hv[k]/hsum;
                of2 << k*RESOLUTION << " " << c << endl;
            }
            of1.close();   of2.close();
        #endif
            
        RCLCPP_INFO(this->get_logger(), "Monitor closed.\n");

        dolog("Monitor closed");

        #if EXTENDED_STAGE
        sleep(5);
        char cmd[80];
        sprintf(cmd, "mv ~/.ros/stage-000003.png %s/%s_stage.png", path4.c_str(),strnow);
        system(cmd);
        RCLCPP_INFO(this->get_logger(), "%s\n",cmd);
        RCLCPP_INFO(this->get_logger(), "Screenshot image copied.\n");
        sleep(3);  
        dolog("Snapshots done");
        #endif
    }

    void dolog(const char *str) {
        if (logfile) {
            fprintf(logfile,"%s\n",str);
            fflush(logfile);
        }
    }

    double get_last_goal_reached(int k)
    {
        pthread_mutex_lock(&lock_last_goal_reached);
        double r = last_goal_reached[k];
        pthread_mutex_unlock(&lock_last_goal_reached);
        return r;
    }

    void set_last_goal_reached(int k, double val)
    {
        pthread_mutex_lock(&lock_last_goal_reached);
        last_goal_reached[k] = val;
        pthread_mutex_unlock(&lock_last_goal_reached);
    }


    void resultsCB(std_msgs::msg::Int16MultiArray::ConstSharedPtr msg) 
    {
        dolog("resultsCB - begin");

        std::vector<signed short>::const_iterator it = msg->data.begin();    
        
        std::vector<int> vresults;
        
        vresults.clear();
        
        for (size_t k=0; k<msg->data.size(); k++) {
            vresults.push_back(*it); it++;
        }

        int id_robot = vresults[0];  // robot sending the message
        int msg_type = vresults[1];  // message type

        switch(msg_type) {
            case INITIALIZE_MSG_TYPE:
            {
            if (initialize && vresults[2]==1){ 
                if (init_robots[id_robot] == false){   //receive init msg: "ID,msg_type,1"
                    RCLCPP_INFO(this->get_logger(), "Robot [ID = %d] is Active!\n", id_robot);
                    init_robots[id_robot] = true;
                    
                    //Patch D.Portugal (needed to support other simulators besides Stage):
                    double current_time = this->get_clock()->now().seconds();
                    //initialize last_goal_reached:
                    set_last_goal_reached(id_robot,current_time);
                    
                    cnt++;
                } 
                if (cnt==teamsize){
                    
                    // check if robots need to travel to starting positions
                    // TODO GOECKNER - temporarily disabled
                    /*
                    while(goto_start_pos){ //if or while (?)
                        
                        patrolling_sim::GoToStartPosSrv::Request Req;
                        Req.teamsize.data = teamsize;
                        Req.sleep_between_goals.data = 20; //time in secs to wait before sending goals to each different robot
                        patrolling_sim::GoToStartPosSrv::Response Rep;	
                        
                        RCLCPP_INFO(this->get_logger(), "Sending all robots to starting position.");
                        
                        if (!ros::service::call("/GotoStartPosSrv", Req, Rep)){ //blocking call
                            RCLCPP_ERROR(this->get_logger(), "Error invoking /GotoStartPosSrv.");	
                            RCLCPP_ERROR(this->get_logger(), "Sending robots to initial position failed.");
                            rclcpp::shutdown(); //make sense for while implementation
                            return;
                            
                        }else{
                            goto_start_pos = false;
                            system("rosnode kill GoToStartPos &");  //we don't need the service anymore.
                        }               
                        
                    }
                    */
                        
                    RCLCPP_INFO(this->get_logger(), "All Robots GO!\n");
                    initialize = false;
                        
                    //Clock Reset:
                    time_zero = this->get_clock()->now().seconds();
                    last_report_time = time_zero; 
                                        
                    time (&real_time_zero);
                    RCLCPP_INFO(this->get_logger(), "Time zero = %.1f (sim) = %lu (real) \n", time_zero,(long)real_time_zero);

                    std_msgs::msg::Int16MultiArray msg;  // -1,msg_type,100,0,0
                    msg.data.clear();
                    msg.data.push_back(-1);
                    msg.data.push_back(INITIALIZE_MSG_TYPE);
                    msg.data.push_back(100);  // Go !!!
                    results_pub->publish(msg);
                    // ros::spinOnce();      
                    
                    }
                }
                
            //}
            break;
            }
            
            case TARGET_REACHED_MSG_TYPE:
            {
                //goal sent by a robot during the experiment [ID,msg_type,vertex,intention,0]
                if (initialize==false){ 
                    goal = vresults[2];
                    RCLCPP_INFO(this->get_logger(), "Robot %d reached Goal %d.\n", id_robot, goal); 
                    fflush(stdout);
                    goal_reached = true;
                    update_stats(id_robot, goal);
                    // ros::spinOnce();
                }
                break;
            }
            
            case INTERFERENCE_MSG_TYPE:
            {
                //interference: [ID,msg_type]
                if (initialize==false){
                    RCLCPP_INFO(this->get_logger(), "Robot %d sent interference.\n", id_robot); 
                    interference_cnt++;
                    // ros::spinOnce();
                }
                break;
            }
        }

        dolog("resultsCB - end");

    }

    void finish_simulation (){ //-1,msg_type,999,0,0
        RCLCPP_INFO(this->get_logger(), "Sending stop signal to patrol agents.");
        std_msgs::msg::Int16MultiArray msg;  
        msg.data.clear();
        msg.data.push_back(-1);
        msg.data.push_back(INITIALIZE_MSG_TYPE);
        msg.data.push_back(999);  // end of the simulation
        results_pub->publish(msg);
        // ros::spinOnce();  

        #if EXTENDED_STAGE  
        RCLCPP_INFO(this->get_logger(), "Taking a screenshot of the simulator...");
        std_msgs::String ss;
        ss.data = "screenshot";
        screenshot_pub.publish(ss);
        #endif
        
        // ros::spinOnce();  
    }

    // return the median value in a vector of size "dimension" floats pointed to by a
    double Median( double *a, uint dimension )
    {
        uint table_size = dimension/2;   
        if(dimension % 2 != 0){ //odd
            table_size++; 
        }   
        if (table_size==0) {table_size = 1;}
        
        double left[table_size], right[table_size], median, *p;
        unsigned char nLeft, nRight;

        // pick first value as median candidate
        p = a;
        median = *p++;
        nLeft = nRight = 1;

        for(;;)
        {
            // get next value
            double val = *p++;

            // if value is smaller than median, append to left heap
            if( val < median )
            {
                // move biggest value to the heap top
                unsigned char child = nLeft++, parent = (child - 1) / 2;
                while( parent && val > left[parent] )
                {
                    left[child] = left[parent];
                    child = parent;
                    parent = (parent - 1) / 2;
                }
                left[child] = val;

                // if left heap is full
                if( nLeft == table_size )
                {
                    // for each remaining value
                    for( unsigned char nVal = dimension - (p - a); nVal; --nVal )
                    {
                        // get next value
                        val = *p++;

                        // if value is to be inserted in the left heap
                        if( val < median )
                        {
                            child = left[2] > left[1] ? 2 : 1;
                            if( val >= left[child] )
                                median = val;
                            else
                            {
                                median = left[child];
                                parent = child;
                                child = parent*2 + 1;
                                while( child < table_size )
                                {
                                    if( child < table_size-1 && left[child+1] > left[child] )
                                        ++child;
                                    if( val >= left[child] )
                                        break;
                                    left[parent] = left[child];
                                    parent = child;
                                    child = parent*2 + 1;
                                }
                                left[parent] = val;
                            }
                        }
                    }
                    return median;
                }
            }

            // else append to right heap
            else
            {
                // move smallest value to the heap top
                unsigned char child = nRight++, parent = (child - 1) / 2;
                while( parent && val < right[parent] )
                {
                    right[child] = right[parent];
                    child = parent;
                    parent = (parent - 1) / 2;
                }
                right[child] = val;

                // if right heap is full
                if( nRight == 14 )
                {
                    // for each remaining value
                    for( unsigned char nVal = dimension - (p - a); nVal; --nVal )
                    {
                        // get next value
                        val = *p++;

                        // if value is to be inserted in the right heap
                        if( val > median )
                        {
                            child = right[2] < right[1] ? 2 : 1;
                            if( val <= right[child] )
                                median = val;
                            else
                            {
                                median = right[child];
                                parent = child;
                                child = parent*2 + 1;
                                while( child < table_size )
                                {
                                    if( child < 13 && right[child+1] < right[child] )
                                        ++child;
                                    if( val <= right[child] )
                                        break;
                                    right[parent] = right[child];
                                    parent = child;
                                    child = parent*2 + 1;
                                }
                                right[parent] = val;
                            }
                        }
                    }
                    return median;
                }
            }
        }
    }


    uint calculate_patrol_cycle ( int *nr_visits, uint dimension ){
        dolog("    calculate_patrol_cycle - begin");
        uint result = INT_MAX;
        uint imin=0;
        for (uint i=0; i<dimension; i++){
            if ((uint)nr_visits[i] < result){
            result = nr_visits[i]; imin=i;
            }
        }
        //printf("  --- complete patrol: visits of %d : %d\n",imin,result);
        dolog("    calculate_patrol_cycle - end");
        return result;  
    }

    void scenario_name(char* name, const char* graph_file, const char* teamsize_str)
    {
        uint i, start_char=0, end_char = strlen(graph_file)-1;
        
        for (i=0; i<strlen(graph_file); i++){
            if(graph_file[i]=='/' && i < strlen(graph_file)-1){
                start_char = i+1;
            }
            
            if(graph_file[i]=='.' && i>0){
                end_char = i-1;
                break;
            }
        }
        
        for (i=start_char; i<=end_char; i++){
            name [i-start_char] = graph_file [i];
            if (i==end_char){
                name[i-start_char+1] = '\0';
            }
        }
        
        strcat(name,"_");
        strcat(name,teamsize_str);
    }

    //write_results to file
    void write_results (double *avg_idleness, double *stddev_idleness, int *number_of_visits, uint complete_patrol, uint dimension, 
                        double worst_avg_idleness, double avg_graph_idl, double median_graph_idl, double stddev_graph_idl, 
                        double min_idleness, double gavg, double gstddev, double max_idleness,
                        uint interference_cnt, uint tot_visits, float avg_visits,
                        const char* graph_file, const char* teamsize_str, 
                        double duration, double real_duration, double comm_delay,
                    string filename) {

        dolog("write_results - begin");

        FILE *file;
    
        RCLCPP_INFO(this->get_logger(), "writing to file %s\n",filename.c_str());
        // RCLCPP_INFO(this->get_logger(), "graph file %s\n",graph_file);
            
        file = fopen (filename.c_str(),"a");
        
        //fprintf(file,"%i\n%i\n%i\n\n",num_nos,largura(),altura());
        fprintf(file, "\nComplete Patrol Cycles:\t%u\n\n", complete_patrol);
        fprintf(file, "Vertex\tAvg Idl\tStdDev Idl\t#Visits\n");
        for (uint i=0; i<dimension; i++){
            fprintf(file, "%u\t%.1f\t%.1f\t%d\n", i, avg_idleness[i], stddev_idleness[i], number_of_visits[i] );
        }

        fprintf(file,"\nNode idleness\n");
        fprintf(file,"   worst_avg_idleness (graph) = %.1f\n", worst_avg_idleness);
        fprintf(file,"   avg_idleness (graph) = %.1f\n", avg_graph_idl);
        fprintf(file,"   median_idleness (graph) = %.1f\n", median_graph_idl);
        fprintf(file,"   stddev_idleness (graph) = %.1f\n", stddev_graph_idl);

        fprintf(file,"\nGlobal idleness\n");
        fprintf(file,"   min = %.1f\n", min_idleness);
        fprintf(file,"   avg = %.1f\n", gavg);
        fprintf(file,"   stddev = %.1f\n", gstddev);
        fprintf(file,"   max = %.1f\n", max_idleness);

        fprintf(file,"\nInterferences\t%u\nInterference rate\t%.2f\nVisits\t%u\nAvg visits per node\t%.1f\nTime Elapsed\t%.1f\nReal Time Elapsed\t%.1f\nComm delay: %.2f\n",
        interference_cnt,(float)interference_cnt/duration*60,tot_visits,avg_visits,duration,real_duration,comm_delay);
        
        fprintf(file,"----------------------------------------------------------------------------------------------------------------------------------------------------------------\n\n\n");    
        
        
        fclose(file); /*done!*/  

        dolog("write_results - end");

    }

    bool check_dead_robots() {

        dolog("  check_dead_robots - begin");

        double current_time = this->get_clock()->now().seconds();
        bool r=false;
        for (size_t i=0; i<teamsize; i++) {
            double l = get_last_goal_reached(i);
            double delta = current_time - l;
            // RCLCPP_INFO(this->get_logger(), "DEBUG dead robot: %d   %.1f - %.1f = %.1f\n",i,current_time,l,delta);
            if (delta>DEAD_ROBOT_TIME*0.75) {
                RCLCPP_INFO(this->get_logger(), "Robot %lu: dead robot - delta = %.1f / %.1f \n",i,delta,DEAD_ROBOT_TIME);
                system("play -q beep.wav");
            }
            if (delta>DEAD_ROBOT_TIME) {
                // RCLCPP_INFO(this->get_logger(), "Dead robot %d. Time from last goal reached = %.1f\n",i,delta);
                r=true;
                break;
            }
        }

        dolog("  check_dead_robots - end");

        return r;
    }


    // update stats after robot 'id_robot' visits node 'goal'
    void update_stats(int id_robot, int goal) {

        dolog("  update_stats - begin");

        
    //   RCLCPP_INFO(this->get_logger(), "last_visit [%d] = %.1f\n", goal, last_visit [goal]);
        double current_time = this->get_clock()->now().seconds();

        RCLCPP_INFO(this->get_logger(), "Robot %d reached goal %d (current time: %.2f, alg: %s, nav: %s)\n", id_robot, goal, current_time, algorithm.c_str(), nav_mod.c_str());
                
        double last_visit_temp = current_time - time_zero; //guarda o valor corrente
        number_of_visits [goal] ++;
        
        set_last_goal_reached(id_robot,current_time);

        RCLCPP_INFO(this->get_logger(), "   nr_of_visits = %d -", number_of_visits [goal]);

        if (number_of_visits [goal] == 0) {
            avg_idleness [goal] = 0.0; stddev_idleness[goal] = 0.0;
            total_0 [goal] = 0.0; total_1 [goal] = 0.0;  total_2 [goal] = 0.0;
        }
        else { // if (number_of_visits [goal] > 0) {

            current_idleness [goal] = last_visit_temp - last_visit [goal];
        
            if (current_idleness [goal] > max_idleness)
                max_idleness=current_idleness [goal];
            if (current_idleness [goal] < min_idleness || min_idleness<0.1)
                min_idleness=current_idleness [goal];
            
            // global stats
            gT0++; gT1 += current_idleness[goal]; gT2 += current_idleness[goal]*current_idleness[goal];
        
            // node stats
            total_0 [goal] += 1.0; total_1 [goal] += current_idleness [goal];  total_2 [goal] += current_idleness [goal]*current_idleness [goal];
            avg_idleness [goal] = total_1[goal]/total_0[goal]; 
            stddev_idleness[goal] = 1.0/total_0[goal] * sqrt(total_0[goal]*total_2[goal]-total_1[goal]*total_1[goal]); 
            
            RCLCPP_INFO(this->get_logger(), " idl current = %.2f, ", current_idleness[goal]);
            RCLCPP_INFO(this->get_logger(), " avg = %.1f, stddev = %.1f,", avg_idleness [goal], stddev_idleness[goal]);
            RCLCPP_INFO(this->get_logger(), " max = %.1f - interf = %d\n", max_idleness,interference_cnt);

            // save data in idleness file
            fprintf(idlfile,"%.1f;%d;%d;%.1f;%d\n",current_time,id_robot,goal,current_idleness[goal],interference_cnt);
            fflush(idlfile);

    #if SAVE_HYSTOGRAMS
            // compute values for hystograms
            int b = (int)(current_idleness[goal]/RESOLUTION);
            if (b<hn) {
            hv[b]++; hsum++;
            }
    #endif

        }

        complete_patrol = calculate_patrol_cycle ( number_of_visits, dimension );                
        RCLCPP_INFO(this->get_logger(), "   complete patrol cycles = %d\n", complete_patrol); 
        
        // Compute node with highest current idleness
        size_t hnode; double hidl=0;
        for (size_t i=0; i<dimension; i++) {
            double cidl = last_visit_temp - last_visit [i];
            if (cidl>hidl) {
                hidl=cidl; hnode=i;
            }
        }
        RCLCPP_INFO(this->get_logger(), "   highest current idleness: node %lu idl %.1f\n\n",hnode,hidl);
        
        last_visit [goal] = last_visit_temp;
        
        goal_reached = false;

        dolog("  update_stats - end");

    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PatrolMonitor> node = std::make_shared<PatrolMonitor>();
    rclcpp::spin(node);
    node->on_experiment_end();
    rclcpp::shutdown();
    return 0; 
}

