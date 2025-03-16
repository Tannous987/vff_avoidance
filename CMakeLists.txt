cmake_minimum_required(VERSION 3.5)
project(vff_avoidance)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include headers
include_directories(include)

# Declare executable
add_executable(avoidance_vff src/avoidance_vff_main.cpp src/vff_avoidance/AvoidanceNode.cpp)
ament_target_dependencies(avoidance_vff rclcpp sensor_msgs geometry_msgs visualization_msgs)

# Install executables and launch files
install(TARGETS
  avoidance_vff
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs visualization_msgs)
ament_package()
