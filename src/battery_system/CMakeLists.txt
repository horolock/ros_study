cmake_minimum_required(VERSION 3.8)
project(battery_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Build
add_executable(battery_node src/battery.cpp)
add_executable(led_panel src/led_panel.cpp)

# Add Dependencies
ament_target_dependencies(battery_node 
  rclcpp 
  my_robot_interfaces
)

ament_target_dependencies(led_panel 
  rclcpp 
  my_robot_interfaces 
)

# Install
install(TARGETS 
  battery_node
  led_panel 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
