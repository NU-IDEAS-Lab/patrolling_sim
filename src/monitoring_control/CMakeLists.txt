cmake_minimum_required(VERSION 3.8)
project(monitoring_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(patrolling_sim REQUIRED)
find_package(patrolling_sim_interfaces REQUIRED)

# Build the monitor executable and link against dependencies.
# add_executable(monitor src/monitor.cpp)
# target_include_directories(monitor PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(monitor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   monitor
#   rclcpp
#   tf2
#   std_msgs
#   patrolling_sim_interfaces
# )

add_executable(monitor src/monitor.cpp)
ament_target_dependencies(monitor rclcpp std_msgs patrolling_sim patrolling_sim_interfaces)


install(TARGETS monitor
  DESTINATION lib/${PROJECT_NAME})

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
