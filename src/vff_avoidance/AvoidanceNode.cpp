#include "vff_avoidance/AvoidanceNode.hpp"
#include <cmath>

// Constructor for the AvoidanceNode class
AvoidanceNode::AvoidanceNode() : Node("avoidance_node") {
  // Declare parameters for topics and robot behavior
  this->declare_parameter("laser_topic", "scan");
  this->declare_parameter("cmd_vel_topic", "cmd_vel");
  this->declare_parameter("marker_topic", "visualization_marker_array");
  this->declare_parameter("max_linear_speed", 0.5);
  this->declare_parameter("max_angular_speed", 1.5);
  this->declare_parameter("obstacle_threshold", 1.0);
  this->declare_parameter("attractive_gain", 1.0);
  this->declare_parameter("repulsive_gain", 1.0);
  this->declare_parameter("stale_data_timeout", 2.0);  // Timeout for outdated sensor data
  
  // Marker scaling parameters for visualization
  this->declare_parameter("marker_scale_x", 0.02);  // Shaft diameter   
  this->declare_parameter("marker_scale_y", 0.03);  // Head diameter
  this->declare_parameter("marker_scale_z", 0.03);  // Head length              
   
  // Parameters related to laser scan angles
  this->declare_parameter("angle_min", -M_PI / 2);  // Minimum angle (-90 degrees in radians)
  this->declare_parameter("angle_increment", M_PI / 180);  // Angle step size (1 degree in radians)

  // Subscribers and Publishers
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("laser_topic").as_string(), 10,
      std::bind(&AvoidanceNode::laserCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      this->get_parameter("marker_topic").as_string(), 10);

  // Timer for control loop execution at 20Hz (every 50ms)
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20Hz
      std::bind(&AvoidanceNode::controlLoop, this));

  last_scan_time_ = this->now();  // Initialize to prevent outdated scan issues
}

// Callback function to handle incoming laser scan data
void AvoidanceNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  laser_ranges_ = msg->ranges;  // Store the received distance data
  latest_scan_ = msg;  // Store the latest scan message
  last_scan_time_ = this->now(); // Update last received scan timestamp
}

// Main control loop that computes movement based on sensor data
void AvoidanceNode::controlLoop() {
  // Check if laser scan data is available and not outdated
  if (laser_ranges_.empty() || (this->now() - last_scan_time_).seconds() > this->get_parameter("stale_data_timeout").as_double()) {
    return;  // Exit if no valid data is available
  }

  // Attractive force vector (always moves forward)
  double attractive_x = this->get_parameter("attractive_gain").as_double();
  double attractive_y = 0.0;

  // Compute repulsive force based on the closest detected obstacle
  double min_distance = std::numeric_limits<double>::max();
  double min_angle = 0.0;
  for (size_t i = 0; i < laser_ranges_.size(); i++) {
    double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
    double distance = laser_ranges_[i];
    if (distance < min_distance) {  // Track closest obstacle
      min_distance = distance;
      min_angle = angle;
    }
  }

  // Compute repulsive force components
  double repulsive_x = 0.0, repulsive_y = 0.0;
  if (min_distance < this->get_parameter("obstacle_threshold").as_double()) {
    double repulsive_magnitude = this->get_parameter("repulsive_gain").as_double() / min_distance;
    repulsive_x = -repulsive_magnitude * std::cos(min_angle);
    repulsive_y = -repulsive_magnitude * std::sin(min_angle);
  }

  // Compute resultant movement vector
  double result_x = attractive_x + repulsive_x;
  double result_y = attractive_y + repulsive_y;

  // Ensure values are valid
  if (std::isnan(result_x) || std::isnan(result_y)) {
    RCLCPP_WARN(this->get_logger(), "NaN values detected in resultant vector.");
    return;
  }

  // Compute linear and angular velocities
  double linear_speed = std::min(this->get_parameter("max_linear_speed").as_double(), std::hypot(result_x, result_y));
  double angular_speed = std::atan2(result_y, result_x);

  // Publish velocity command
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = linear_speed;
  cmd_vel->angular.z = angular_speed;
  cmd_vel_pub_->publish(std::move(cmd_vel));

  // Publish visualization markers for debugging
  publishMarkers(result_x, result_y, repulsive_x, repulsive_y);
}

// Function to visualize vectors using markers
void AvoidanceNode::publishMarkers(double result_x, double result_y, double repulsive_x, double repulsive_y) {
  if (marker_pub_->get_subscription_count() == 0) return;  // Avoid unnecessary publishing

  visualization_msgs::msg::MarkerArray marker_array;

  // Clear previous markers
  visualization_msgs::msg::Marker clear_markers;
  clear_markers.header.frame_id = "base_link";
  clear_markers.header.stamp = this->now();
  clear_markers.ns = "vff_vectors";
  clear_markers.id = 0;  // Unique ID for DELETEALL action
  clear_markers.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_markers);

  // Function to create an arrow marker for visualization
  auto createArrowMarker = [this](int id, double x, double y, double r, double g, double b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "vff_vectors";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define arrow scale
    marker.scale.x = this->get_parameter("marker_scale_x").as_double();
    marker.scale.y = this->get_parameter("marker_scale_y").as_double();
    marker.scale.z = this->get_parameter("marker_scale_z").as_double();

    // Define color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    // Define start and end points of the arrow
    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    end.x = x;
    end.y = y;
    marker.points.push_back(start);
    marker.points.push_back(end);

    return marker;
  };

  // Create and publish markers for visualization
  marker_array.markers.push_back(createArrowMarker(1, this->get_parameter("attractive_gain").as_double(), 0.0, 0.0, 0.0, 1.0));  // Blue: Attractive force
  marker_array.markers.push_back(createArrowMarker(2, repulsive_x, repulsive_y, 1.0, 0.0, 0.0));  // Red: Repulsive force
  marker_array.markers.push_back(createArrowMarker(3, result_x, result_y, 0.0, 1.0, 0.0));  // Green: Resultant force

  marker_pub_->publish(marker_array);
}
