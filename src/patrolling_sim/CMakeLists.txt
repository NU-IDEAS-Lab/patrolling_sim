cmake_minimum_required(VERSION 3.8)
project(patrolling_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

# Build GetGraph library.
add_library(GetGraph
  src/getgraph.cpp
)
ament_target_dependencies(GetGraph rclcpp)

# Install include files.
install(
  DIRECTORY include/patrolling_sim
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration/parameter files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install map files.
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# ament_export_include_directories(include)
ament_export_targets(export_getgraph HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)  

install(
  TARGETS GetGraph
  EXPORT export_getgraph
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
