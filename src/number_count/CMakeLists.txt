cmake_minimum_required(VERSION 3.8)
project(number_count)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Build
add_executable(number_publisher src/number_publisher.cpp)
add_executable(number_counter src/number_counter.cpp)
add_executable(reset_client src/reset_client.cpp)
add_executable(hw_status_publisher src/hw_status_publisher.cpp)

# Link
ament_target_dependencies(number_publisher 
  rclcpp 
  std_msgs
)

ament_target_dependencies(number_counter 
  rclcpp 
  std_msgs 
  std_srvs 
)

ament_target_dependencies(reset_client 
  rclcpp 
  std_msgs 
  std_srvs 
)

ament_target_dependencies(hw_status_publisher 
  rclcpp 
  my_robot_interfaces
)

# Install
install(TARGETS 
  number_publisher
  number_counter 
  reset_client 
  hw_status_publisher 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
