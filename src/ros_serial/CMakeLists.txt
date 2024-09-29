cmake_minimum_required(VERSION 3.8)
project(ros_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED 
  src/serial_interface.cpp
  include/serial_interface.hpp
)

target_sources(${PROJECT_NAME} PUBLIC 
  src/serial_linux.cpp
  src/serial_list_ports.cpp
)
target_link_libraries(${PROJECT_NAME} rt pthread)

target_include_directories(${PROJECT_NAME} PUBLIC include)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
)

install(FILES include/serial_interface.hpp
  DESTINATION include/ros_serial
)

ament_package()
