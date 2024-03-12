#!/bin/bash

# ros2 doctor --report

exec 3>&1 4>&2
trap 'exec 2>&4 1>&3' 0 1 2 3
exec 1>out/40ky3q01_no_attrition_obs30_comms10_NERF_GBS.out 2>&1
# Everything below will go to the file 'log.out':

source ./install/setup.bash
source /usr/share/gazebo/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 doctor --report

AGENTS=6
RUNTIME=1800
STEP_SIZE=0.02
UPDATE_RATE=100.0
PARAMS_FILE="/home/anthony/dev/patrolling_sim/params_obs30_comms10.yml"
OUTPUT_FILE="$HOME/papers/iros2024/sim_results/40ky3q01_no_attrition_obs30_comms10_NERF_GBS.zarr"
ATTRITION_TIMES="-1.0,-1.0"
# ATTRITION_TIMES="300.0,1100.0"
# MODEL_DIR="/mnt/c/Users/Anthony/Desktop/patrolling_policies/cumberland/rmappo/6SharedAttritionYesCommsNoSkipSyncGNN/wandb/run-20240223_175018-9k9znk4e/files"
# MODEL_DIR="/mnt/c/Users/Anthony/Desktop/patrolling_policies/random9/rmappo/graphSAGEIdInGraphObsOnlyRegenerate20RandIds/wandb/run-20240225_174710-it4e1y0w/files"
# MODEL_DIR="/home/anthony/papers/aamas2024/policies/cumberland/rmappo/graphSAGEIdInGraphObsOnly/wandb/run-20240225_164008-vcwaqqz6/files"
# MODEL_DIR="/home/anthony/papers/aamas2024/policies/milwaukee/mappo/nbrScoringDepth10JKAggrAdd/wandb/run-20240305_221717-8f69l6zc/files"

# MODEL_DIR="/home/anthony/papers/aamas2024/policies/milwaukee/mappo/partialObsDepth10JKAggrAdd/wandb/run-20240308_174627-n1j1aiut/files"

MODEL_DIR="/home/anthony/dev/patrolling_zoo/onpolicy/scripts/results/Patrolling/milwaukee/mappo/partialObs400Depth10JKAggrAddComms0.1LR0.0003/wandb/run-20240311_011045-40ky3q01/files"

for i in {1..3}; do

    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_ahpa algorithm_name:=MARL agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_ahpa algorithm_name:=AHPA agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_base algorithm_name:=CBLS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTAGreedy agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"
    # ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTASSIPart agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"
    ros2 launch patrolling_sim simulation.launch.py sim:=flatland step_size:=$STEP_SIZE update_rate:=$UPDATE_RATE algorithm_pkg:=patrol_algorithms_base algorithm_name:=GBS agent_count:=$AGENTS use_rviz:=false headless:=true runtime:=$RUNTIME output:="$OUTPUT_FILE" attrition_times:="$ATTRITION_TIMES" agent_policy_dir:="$MODEL_DIR" params_file:="$PARAMS_FILE"

done