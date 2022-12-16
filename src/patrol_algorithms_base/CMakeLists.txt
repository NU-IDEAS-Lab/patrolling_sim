cmake_minimum_required(VERSION 3.8)
project(patrol_algorithms_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(patrolling_sim_interfaces REQUIRED)

include_directories(src include)

# Temporarily hide all warnings.
add_definitions(-w)

########################################
# Add libraries.
########################################

add_library(PatrolAgent
  src/PatrolAgent.cpp
  src/getgraph.cpp
  src/algorithms.cpp
  src/config.cpp
)
ament_target_dependencies(PatrolAgent rclcpp tf2 tf2_ros geometry_msgs std_msgs nav_msgs nav2_msgs patrolling_sim_interfaces)

# add_library(SSIPatrolAgent
#   src/SSIPatrolAgent.cpp
# )
# ament_target_dependencies(SSIPatrolAgent rclcpp)

########################################
# Add an executable for each algorithm.
########################################

# add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
# target_link_libraries(Conscientious_Reactive PatrolAgent)
 
# add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
# target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent)

# add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
# target_link_libraries(Conscientious_Cognitive PatrolAgent)
 
# add_executable(Cyclic src/Cyclic_Agent.cpp)
# target_link_libraries(Cyclic PatrolAgent)

# add_executable(MSP src/MSP_Agent.cpp)
# target_link_libraries(MSP PatrolAgent)

# add_executable(GBS src/GBS_Agent.cpp)
# target_link_libraries(GBS PatrolAgent)

# add_executable(SEBS src/SEBS_Agent.cpp)
# target_link_libraries(SEBS PatrolAgent)

# add_executable(CBLS src/CBLS_Agent.cpp)
# target_link_libraries(CBLS PatrolAgent)

add_executable(Random src/Random_Agent.cpp)
target_link_libraries(Random PatrolAgent)

# add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
# target_link_libraries(DTAGreedy PatrolAgent)

# add_executable(DTASSI src/DTASSI_Agent.cpp)
# target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent)

# add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
# target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent)


# Install libraries.
# ament_export_targets(my_libraryTargets HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS PatrolAgent #SSIPatrolAgent
  # EXPORT my_libraryTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
