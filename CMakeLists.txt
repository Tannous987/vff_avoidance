cmake_minimum_required(VERSION 3.5)  # Specifies the minimum required version of CMake (3.5)
project(vff_avoidance)  # Defines the project name as "vff_avoidance"

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)  # Checks if C++ standard is not already set
  set(CMAKE_CXX_STANDARD 17)  # Sets the C++ standard to C++17 if not already set
endif()  # End of the if condition

# Find dependencies
find_package(ament_cmake REQUIRED)  # Finds the ament_cmake package (required for building ROS 2 packages)
find_package(rclcpp REQUIRED)  # Finds the rclcpp package (required for ROS 2 C++ client library)
find_package(sensor_msgs REQUIRED)  # Finds the sensor_msgs package (provides message types for sensors)
find_package(geometry_msgs REQUIRED)  # Finds the geometry_msgs package (provides message types for geometric data)
find_package(visualization_msgs REQUIRED)  # Finds the visualization_msgs package (provides message types for visualization)

# Include headers
include_directories(include)  # Adds the "include" directory to the list of directories to search for header files

# Declare executable
add_executable(avoidance_vff src/avoidance_vff_main.cpp src/vff_avoidance/AvoidanceNode.cpp)  # Declares the executable "avoidance_vff" with the source files specified
ament_target_dependencies(avoidance_vff rclcpp sensor_msgs geometry_msgs visualization_msgs)  # Links the executable "avoidance_vff" with the dependencies specified (rclcpp, sensor_msgs, geometry_msgs, visualization_msgs)

# Install executables and launch files
install(TARGETS  # Specifies which targets to install
  avoidance_vff  # Declares the executable "avoidance_vff" as a target to install
  DESTINATION lib/${PROJECT_NAME})  # Defines the destination directory for the executable (lib/${PROJECT_NAME})

install(DIRECTORY  # Installs a directory (launch files)
  launch  # Specifies the source directory (launch)
  DESTINATION share/${PROJECT_NAME})  # Defines the destination directory for launch files (share/${PROJECT_NAME})

install(DIRECTORY  # Installs a directory (config files)
  config  # Specifies the source directory (config)
  DESTINATION share/${PROJECT_NAME})  # Defines the destination directory for config files (share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs visualization_msgs)  # Exports the package dependencies so other packages can use them

ament_package()  # Ends the package declaration, ensuring proper handling of dependencies, installation, etc.
