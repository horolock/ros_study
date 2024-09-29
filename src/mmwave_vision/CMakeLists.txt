cmake_minimum_required(VERSION 3.8)
project(mmwave_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set
set (srv_files 
  "srv/MMWaveCommandService.srv"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# START ROS Interfaces
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(mmwave_interfaces 
  ${srv_files}
)
ament_export_dependencies(rosidl_default_generators)
# END ROS Interfaces

add_executable(mmwave_config_controller src/mmwave_config_controller.cpp)

ament_target_dependencies(mmwave_config_controller 
  rclcpp 
)

install(TARGETS 
  mmwave_config_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
