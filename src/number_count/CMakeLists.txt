cmake_minimum_required(VERSION 3.8)
project(number_count)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build
add_executable(number_publisher src/number_publisher.cpp)

# Link
ament_target_dependencies(number_publisher 
  rclcpp 
  std_msgs
)

# Install
install(TARGETS 
  number_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
