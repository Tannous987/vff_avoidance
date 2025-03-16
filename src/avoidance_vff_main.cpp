#include "vff_avoidance/AvoidanceNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}