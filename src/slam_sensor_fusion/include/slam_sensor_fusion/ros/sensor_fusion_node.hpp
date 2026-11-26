#pragma once

#include "slam_sensor_fusion/core/differential_adapter.hpp"
#include "slam_sensor_fusion/core/ekf_2d.hpp"
#include "slam_sensor_fusion/ros/message_converters.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace slam_sensor_fusion::ros_node {

/// @brief Fusion mode governing how a nav_msgs::msg::Odometry topic is fused into Ekf2D.
/// @see docs/architecture/sensor_fusion_node.md, Section 4.
enum class OdomFusionMode {
  kTwist,             ///< Fuse twist.twist (vx, vy, omega) directly.
  kPoseAbsolute,      ///< Fuse pose.pose (x, y, theta) directly in the reference frame.
  kPoseDifferential,  ///< Differentiate consecutive pose.pose into body twist via DifferentialAdapter.
  kPoseAndTwist,      ///< Fuse both absolute pose and body twist.
};

/// @brief Runtime telemetry counters exposed for diagnostics and integration testing.
struct FusionStats {
  std::uint64_t total_updates_accepted{0};
  std::uint64_t total_updates_rejected_outlier{0};
  std::uint64_t total_updates_invalid{0};
  std::uint64_t total_out_of_order_updates{0};
  std::uint64_t total_dropped_stale{0};
  std::uint64_t total_clock_resets{0};
};

/// @brief ROS 2 Humble multi-sensor fusion node wrapping the pure C++/Eigen Ekf2D core.
///
/// Subscribes to configurable IMU, wheel odometry, LiDAR scan-matching odometry, and visual
/// odometry topics; transforms sensor-frame measurements into base_link via TF2; fuses them
/// into Ekf2D; and publishes filtered odometry, a bounded RViz Path, and optional TF2 edges.
///
/// Thread safety: internal filter state and adapters are guarded by an internal mutex, making
/// the node safe under both SingleThreadedExecutor and MultiThreadedExecutor.
/// @see docs/architecture/sensor_fusion_node.md.
class SensorFusionNode final : public rclcpp::Node {
 public:
  /// @brief Construct the node, declare & validate YAML parameters, and create pub/sub endpoints.
  /// @param options Standard ROS 2 NodeOptions (supports parameter overrides in launch & tests).
  /// @throws std::invalid_argument If any declared ROS parameter fails startup validation.
  explicit SensorFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief Return a thread-safe snapshot of the current 2D EKF state belief.
  /// @return Copy of the current core::EkfState.
  [[nodiscard]] core::EkfState currentState() const;

  /// @brief Return a thread-safe snapshot of cumulative update and health counters.
  /// @return Copy of FusionStats.
  [[nodiscard]] FusionStats stats() const;

  /// @brief Reset the EKF filter, differential adapters, and published path history.
  void resetFilter();

 private:
  struct ImuSensorRuntime {
    std::string name{"imu"};
    bool enabled{true};
    std::string topic{"/imu"};
    bool use_sensor_data_qos{false};
    ImuConversionOptions conversion{};
    double timeout_sec{0.5};
    std::optional<double> last_msg_time_sec;
    bool timed_out_logged{false};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
  };

  struct OdomSensorRuntime {
    std::string name;
    bool enabled{true};
    std::string topic;
    bool use_sensor_data_qos{false};
    OdomFusionMode mode{OdomFusionMode::kTwist};
    OdomConversionOptions conversion{};
    double timeout_sec{0.5};
    std::optional<double> last_msg_time_sec;
    bool timed_out_logged{false};
    std::unique_ptr<core::DifferentialAdapter> diff_adapter;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
  };

  enum class TimeAdvanceAction {
    kReady,
    kDropStale,
    kResetTriggered,
  };

  void loadAndValidateParameters();
  [[nodiscard]] OdomSensorRuntime declareOdomSensorParams(
      const std::string& prefix,
      const std::string& default_topic,
      const std::string& default_mode,
      const std::vector<double>& default_pose_var,
      const std::vector<double>& default_twist_var,
      const std::vector<double>& default_diff_var,
      double default_timeout_sec);

  [[nodiscard]] static OdomFusionMode parseFusionMode(const std::string& mode_str,
                                                      const std::string& param_name);
  [[nodiscard]] rclcpp::QoS makeSubscriptionQoS(bool use_sensor_data_qos) const;

  void handleImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void handleOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg,
                          OdomSensorRuntime& sensor);
  void handleHealthTimer();

  [[nodiscard]] std::optional<Extrinsic2D> lookupExtrinsicToBase(
      const std::string& raw_sensor_frame_id,
      const rclcpp::Time& stamp);

  [[nodiscard]] TimeAdvanceAction advanceFilterToTimestampLocked(
      double msg_time_sec,
      const std::optional<core::Pose2DMeasurement>& initial_pose_hint);

  void recordUpdateResultLocked(const core::UpdateResult& result, const std::string& sensor_name);
  void publishOutputsLocked(const rclcpp::Time& stamp);
  void resetFilterLocked();

  // Frame & output settings
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  std::string output_odom_topic_{"/slam/odometry/filtered"};
  std::string output_path_topic_{"/slam/path/filtered"};
  bool publish_tf_{false};
  bool publish_path_{true};
  std::size_t max_path_length_{2000};
  double path_min_distance_m_{0.05};
  double tf_lookup_timeout_sec_{0.02};
  double status_log_period_sec_{5.0};
  double out_of_order_tolerance_sec_{0.05};
  double clock_reset_threshold_sec_{1.0};

  // Core estimator & synchronization
  mutable std::mutex mutex_;
  std::unique_ptr<core::Ekf2D> ekf_;
  FusionStats stats_{};

  // Sensor runtimes
  ImuSensorRuntime imu_sensor_{};
  OdomSensorRuntime wheel_odom_sensor_{};
  OdomSensorRuntime lidar_odom_sensor_{};
  OdomSensorRuntime visual_odom_sensor_{};

  // Bounded path history for RViz
  std::deque<geometry_msgs::msg::PoseStamped> path_poses_;

  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace slam_sensor_fusion::ros_node
