#ifndef VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_
#define VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class AvoidanceNode : public rclcpp::Node {
public:
  AvoidanceNode();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlLoop();
  void publishMarkers(double result_x, double result_y,double repulsive_x, double repulsive_y);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> laser_ranges_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;  // Store the latest scan message
  double max_linear_speed_;
  double max_angular_speed_;
  double obstacle_threshold_;
  double attractive_gain_;
  double repulsive_gain_;
  rclcpp::Time last_scan_time_; 
};

#endif  // VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_