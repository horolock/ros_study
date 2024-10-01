cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(msg_files 
  "msg/HardwareStatus.msg"
  "msg/LEDPanelState.msg"
)

set(srv_files
  "srv/ComputeRectangleArea.srv"
  "srv/LEDCommand.srv"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  ${srv_files}
)

ament_package()
