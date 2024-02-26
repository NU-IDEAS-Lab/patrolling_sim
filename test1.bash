#!/bin/bash

# ros2 doctor --report

# exec 3>&1 4>&2
# trap 'exec 2>&4 1>&3' 0 1 2 3
# exec 1>test1_try4.out 2>&1
# Everything below will go to the file 'log.out':

# ros2 doctor --report

source ./install/setup.bash
source /usr/share/gazebo/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


AGENTS=2
RUNTIME=4000
STEP_SIZE=0.005
UPDATE_RATE=400.0
OUTPUT_FILE="$HOME/papers/aamas2024/sim_results/test_debugging.zarr"
ATTRITION_TIMES="-1.0,-1.0"
# ATTRITION_TIMES="300.0,1100.0"
# MODEL_DIR="/mnt/c/Users/Anthony/Desktop/patrolling_policies/cumberland/rmappo/6SharedAttritionYesCommsNoSkipSyncGNN/wandb/run-20240223_175018-9k9znk4e/files"
MODEL_DIR="/mnt/c/Users/Anthony/Desktop/patrolling_policies/random9/rmappo/graphSAGEIdInGraphObsOnlyRegenerate20RandIds/wandb/run-20240225_174710-it4e1y0w/files"

for i in {1..1}; do

    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_ahpa algorithm_name:=MARL agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE algorithm_pkg:=patrol_algorithms_ahpa algorithm_name:=AHPA agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE algorithm_pkg:=patrol_algorithms_base algorithm_name:=CBLS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTAGreedy agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTASSIPart agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE algorithm_pkg:=patrol_algorithms_base algorithm_name:=GBS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"

done