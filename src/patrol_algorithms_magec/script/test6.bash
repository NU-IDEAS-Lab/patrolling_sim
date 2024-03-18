#!/bin/bash

# ros2 doctor --report

exec 3>&1 4>&2
trap 'exec 2>&4 1>&3' 0 1 2 3
exec 1>out/ze1klire_attrition_obsComparison_comms50_run1.out 2>&1
# Everything below will go to the file 'log.out':

source ./install/setup.bash
source /usr/share/gazebo/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 doctor --report

AGENTS=6
RUNTIME=1800
STEP_SIZE=0.02
UPDATE_RATE=100.0
OUTPUT_FILE="$HOME/papers/iros2024/sim_results/ze1klire_attrition_obsComparison_comms50_run1.zarr"
MONITOR_PKG="patrol_algorithms_magec"
# ATTRITION_TIMES="-1.0,-1.0"
ATTRITION_TIMES="300.0,1100.0"
MODEL_DIR="/home/anthony/dev/patrolling_zoo/onpolicy/scripts/results/Patrolling/milwaukee/mappo/partialObs400Depth10JKAggrAddComms0.1LR0.0003/wandb/run-20240314_141826-ze1klire/files"

for i in {1..3}; do

    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs0_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs10_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs20_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs30_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs40_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obs50_comms50.yml" monitor_pkg:="$MONITOR_PKG"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_magec algorithm_name:=MAGEC agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="/home/anthony/dev/patrolling_sim/params_obsInf_comms50.yml" monitor_pkg:="$MONITOR_PKG"

done