cmake_minimum_required(VERSION 3.8)
project(patrolling_sim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(patrolling_sim_interfaces
  "srv/GoToStartPosSrv.srv"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()
