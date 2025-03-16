#ifndef VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_  // Check if the header file is already included to prevent multiple inclusions
#define VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_  // Define the header file to prevent multiple inclusions

#include <rclcpp/rclcpp.hpp>  // Include the header for ROS 2's C++ client library (rclcpp)
#include <sensor_msgs/msg/laser_scan.hpp>  // Include the message type for laser scan data (sensor_msgs)
#include <geometry_msgs/msg/twist.hpp>  // Include the message type for controlling robot velocity (geometry_msgs)
#include <visualization_msgs/msg/marker.hpp>  // Include the message type for a single marker (visualization_msgs)
#include <visualization_msgs/msg/marker_array.hpp>  // Include the message type for an array of markers (visualization_msgs)

class AvoidanceNode : public rclcpp::Node {  // Declare a class that inherits from rclcpp::Node (ROS 2 node)
public:
  AvoidanceNode();  // Declare the constructor for the AvoidanceNode class

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // Callback function for handling LaserScan messages
  void controlLoop();  // Function to manage the control loop of the avoidance algorithm
  void publishMarkers(double result_x, double result_y, double repulsive_x, double repulsive_y);  // Function to publish visualization markers

  // Subscriber to LaserScan messages
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;  
  // Publisher to send Twist messages for robot velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  
  // Publisher to send MarkerArray messages for visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;  
  // Timer for controlling the periodic execution of tasks (e.g., control loop)
  rclcpp::TimerBase::SharedPtr timer_;  

  // Store laser range data (distance measurements from laser sensor)
  std::vector<float> laser_ranges_;  
  // Store the latest LaserScan message received
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;  
  // Define maximum linear speed for the robot
  double max_linear_speed_;  
  // Define maximum angular speed for the robot
  double max_angular_speed_;  
  // Threshold value to determine if an obstacle is near
  double obstacle_threshold_;  
  // Gain factor for the attractive potential field (attraction to the goal)
  double attractive_gain_;  
  // Gain factor for the repulsive potential field (repulsion from obstacles)
  double repulsive_gain_;  
  // Time of the last received scan
  rclcpp::Time last_scan_time_;  
};

#endif  // End of the include guard to prevent multiple inclusions of this header file
