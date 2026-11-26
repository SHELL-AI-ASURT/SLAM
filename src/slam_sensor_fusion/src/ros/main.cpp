#include "slam_sensor_fusion/ros/sensor_fusion_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<slam_sensor_fusion::ros_node::SensorFusionNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("slam_sensor_fusion_node"), "Fatal startup/runtime error: %s",
                 ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
