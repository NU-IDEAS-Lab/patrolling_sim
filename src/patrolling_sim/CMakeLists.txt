cmake_minimum_required(VERSION 3.5)

project(patrolling_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(actionlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/GoToStartPosSrv.srv
  DEPENDENCIES std_msgs
 )

# ament_export_include_directories(src)
# ament_export_libraries(PatrolAgent SSIPatrolAgent)

include_directories(src)

# Declare a cpp library
add_library(PatrolAgent
  src/PatrolAgent.cpp
  src/getgraph.cpp
  src/algorithms.cpp
  src/config.cpp
)

# Declare a cpp library
add_library(SSIPatrolAgent
  src/SSIPatrolAgent.cpp
)

## Declare a cpp executable
add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
 
## Declare a cpp executable
add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
#  
## Declare a cpp executable 
add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Cognitive PatrolAgent ${catkin_LIBRARIES})
 
## Declare a cpp executable 
add_executable(Cyclic src/Cyclic_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Cyclic PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(MSP src/MSP_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(MSP PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(GBS src/GBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(GBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(SEBS src/SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(SEBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(CBLS src/CBLS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(CBLS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(Random src/Random_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Random PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTAGreedy PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTASSI src/DTASSI_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
 
 
# Results and Monitor Node:
 
## Declare a cpp executable 
add_executable(monitor src/monitor.cpp src/getgraph.cpp)
add_dependencies(monitor ${PROJECT_NAME}_gencpp)
## Specify libraries to link a library or executable target against
target_link_libraries(monitor ${catkin_LIBRARIES})

add_executable(GoToStartPos src/GoToStartPos.cpp)
add_dependencies(GoToStartPos ${PROJECT_NAME}_gencpp)
target_link_libraries(GoToStartPos ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(idlHistogram src/idlHistogram.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(idlHistogram ${catkin_LIBRARIES})