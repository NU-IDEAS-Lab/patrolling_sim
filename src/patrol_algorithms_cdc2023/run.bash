#!/bin/bash

# ros2 doctor --report

exec 3>&1 4>&2
trap 'exec 2>&4 1>&3' 0 1 2 3
exec 1>log.out 2>&1
# Everything below will go to the file 'log.out':

ros2 doctor --report

AGENTS=6
RUNTIME=1800
OUTPUT_FILE="$HOME/idev/patrolling_results/results.zarr"
ATTRITION_TIMES="-1.0,-1.0"

for i in {1..3}; do

    ros2 launch patrolling_sim simulation.launch.py sim:=flatland algorithm_pkg:=patrol_algorithms_cdc2023 algorithm_name:=AHPA agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland algorithm_pkg:=patrol_algorithms_base algorithm_name:=CBLS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTAGreedy agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTASSIPart agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland algorithm_pkg:=patrol_algorithms_base algorithm_name:=GBS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES"

done