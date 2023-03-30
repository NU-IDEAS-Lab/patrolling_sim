#!/bin/bash

# ros2 doctor --report

exec 3>&1 4>&2
trap 'exec 2>&4 1>&3' 0 1 2 3
exec 1>log.out 2>&1
# Everything below will go to the file 'log.out':

ros2 doctor --report

for i in {1..3}; do ros2 launch patrolling_sim simulation.launch.py algorithm_pkg:=patrol_algorithms_base algorithm_name:=CBLS agent_count:=6 use_rviz:=false use_gzclient:=false runtime:=1800; done

for i in {1..3}; do ros2 launch patrolling_sim simulation.launch.py algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTAGreedy agent_count:=6 use_rviz:=false use_gzclient:=false runtime:=1800; done

for i in {1..3}; do ros2 launch patrolling_sim simulation.launch.py algorithm_pkg:=patrol_algorithms_base algorithm_name:=DTASSIPart agent_count:=6 use_rviz:=false use_gzclient:=false runtime:=1800; done

for i in {1..3}; do ros2 launch patrolling_sim simulation.launch.py algorithm_pkg:=patrol_algorithms_cdc2023 algorithm_name:=AHPA agent_count:=6 use_rviz:=false use_gzclient:=false runtime:=1800; done

for i in {1..3}; do ros2 launch patrolling_sim simulation.launch.py algorithm_pkg:=patrol_algorithms_base algorithm_name:=GBS agent_count:=6 use_rviz:=false use_gzclient:=false runtime:=1800; done