#pragma once

#include "slam_sensor_fusion/core/types.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <string_view>

namespace slam_sensor_fusion::ros_node {

/// @brief Planar 2D extrinsic transform T_base_sensor expressing sensor pose in base_link.
/// @see docs/theory/ekf_2d.md, Section 3.6, Eqs. (23)-(25).
struct Extrinsic2D {
  double tx{0.0};   ///< Sensor origin X offset in base_link [m].
  double ty{0.0};   ///< Sensor origin Y offset in base_link [m].
  double yaw{0.0};  ///< Sensor yaw rotation relative to base_link [rad].

  /// @return Identity 2D extrinsic (sensor coincident with base_link).
  [[nodiscard]] static constexpr Extrinsic2D identity() noexcept { return Extrinsic2D{}; }
};

/// @brief Euler angles (ZYX intrinsic: yaw -> pitch -> roll) extracted from a quaternion.
struct RollPitchYaw {
  double roll{0.0};   ///< Rotation about +X [rad].
  double pitch{0.0};  ///< Rotation about +Y [rad].
  double yaw{0.0};    ///< Rotation about +Z, in [-pi, pi) [rad].
};

/// @brief Options controlling ROS sensor_msgs::msg::Imu -> core::Imu2DMeasurement conversion.
struct ImuConversionOptions {
  bool use_yaw{false};
  bool use_yaw_rate{true};
  bool use_linear_accel{true};
  bool compensate_gravity{true};
  bool use_message_covariance{true};
  double fallback_yaw_variance{0.05};
  double fallback_yaw_rate_variance{0.002};
  Eigen::Vector2d fallback_accel_variance{0.25, 0.25};
  double mahalanobis_threshold{13.277};
};

/// @brief Options controlling ROS nav_msgs::msg::Odometry -> core pose/twist conversion.
struct OdomConversionOptions {
  bool use_position{true};
  bool use_yaw{true};
  bool use_vx{true};
  bool use_vy{true};
  bool use_omega{true};
  bool use_message_covariance{true};
  Eigen::Vector3d fallback_pose_variance{0.05, 0.05, 0.02};
  Eigen::Vector3d fallback_twist_variance{0.04, 0.01, 0.02};
  double mahalanobis_threshold{11.345};
};

/// @brief Conversion result pairing a core measurement with covariance fallback diagnostics.
template <typename MeasurementT>
struct ConvertedMeasurement {
  MeasurementT measurement{};
  bool used_fallback_covariance{false};
};

/// @brief Strip leading '/' characters from a ROS frame_id for TF2 compliance (REP-105).
/// @param frame_id Raw frame identifier string.
/// @return Sanitized frame identifier with no leading slashes.
[[nodiscard]] std::string sanitizeFrameId(std::string_view frame_id);

/// @brief Safely extract ZYX roll, pitch, and yaw from a ROS quaternion.
/// @param q Input quaternion message.
/// @return RollPitchYaw if q is finite with norm > 1e-9; std::nullopt otherwise.
[[nodiscard]] std::optional<RollPitchYaw> extractRollPitchYaw(
    const geometry_msgs::msg::Quaternion& q) noexcept;

/// @brief Extract a planar 2D extrinsic T_base_sensor from a TF2 TransformStamped.
/// @param tf_base_from_sensor Transform with header.frame_id = base_link, child = sensor_link.
/// @return Extrinsic2D if transform translation and quaternion are valid; std::nullopt otherwise.
[[nodiscard]] std::optional<Extrinsic2D> extractExtrinsic2D(
    const geometry_msgs::msg::TransformStamped& tf_base_from_sensor) noexcept;

/// @brief Transform a 2D pose measured in sensor_link into base_link: T_w_b = T_w_s * T_b_s^{-1}.
/// @param pose_in_sensor Pose observation of sensor_link in reference frame.
/// @param extrinsic T_base_sensor extrinsic transform.
/// @return Equivalent Pose2DMeasurement of base_link in reference frame.
/// @see docs/theory/ekf_2d.md, Section 3.6, Eq. (23).
[[nodiscard]] core::Pose2DMeasurement transformPose2DToBase(
    const core::Pose2DMeasurement& pose_in_sensor,
    const Extrinsic2D& extrinsic) noexcept;

/// @brief Transform a 2D body-frame twist from sensor_link into base_link with lever-arm physics.
/// @param twist_in_sensor Body twist observed at sensor_link.
/// @param extrinsic T_base_sensor extrinsic transform.
/// @return Equivalent Twist2DMeasurement at base_link with rotated/propagated covariance.
/// @see docs/theory/ekf_2d.md, Section 3.6, Eqs. (24)-(25).
[[nodiscard]] core::Twist2DMeasurement transformTwist2DToBase(
    const core::Twist2DMeasurement& twist_in_sensor,
    const Extrinsic2D& extrinsic) noexcept;

/// @brief Convert a ROS sensor_msgs::msg::Imu message into a base_link core::Imu2DMeasurement.
///
/// Applies optional roll/pitch gravity projection removal (Eq. 13), sensor-to-base_link
/// extrinsic rotation and centripetal lever-arm adjustment, and SPD covariance validation
/// with YAML fallback.
/// @param msg Incoming ROS IMU message.
/// @param options Enabled channel flags, gravity compensation, fallback variances, and gating.
/// @param extrinsic T_base_sensor extrinsic transform (identity when imu_link == base_link).
/// @return Converted Imu2DMeasurement, or std::nullopt if required fields are non-finite.
[[nodiscard]] std::optional<ConvertedMeasurement<core::Imu2DMeasurement>> convertImuMessage(
    const sensor_msgs::msg::Imu& msg,
    const ImuConversionOptions& options,
    const Extrinsic2D& extrinsic = Extrinsic2D::identity()) noexcept;

/// @brief Convert the pose portion of nav_msgs::msg::Odometry into a base_link Pose2DMeasurement.
/// @param msg Incoming ROS Odometry message.
/// @param options Enabled pose channels, fallback variances, and gating threshold.
/// @param extrinsic T_base_sensor extrinsic transform (identity when child_frame_id == base_link).
/// @return Converted Pose2DMeasurement, or std::nullopt if pose fields are non-finite/invalid.
[[nodiscard]] std::optional<ConvertedMeasurement<core::Pose2DMeasurement>> convertOdomPoseMessage(
    const nav_msgs::msg::Odometry& msg,
    const OdomConversionOptions& options,
    const Extrinsic2D& extrinsic = Extrinsic2D::identity()) noexcept;

/// @brief Convert the twist portion of nav_msgs::msg::Odometry into a base_link Twist2DMeasurement.
/// @param msg Incoming ROS Odometry message.
/// @param options Enabled twist channels, fallback variances, and gating threshold.
/// @param extrinsic T_base_sensor extrinsic transform (identity when child_frame_id == base_link).
/// @return Converted Twist2DMeasurement, or std::nullopt if twist fields are non-finite.
[[nodiscard]] std::optional<ConvertedMeasurement<core::Twist2DMeasurement>>
convertOdomTwistMessage(const nav_msgs::msg::Odometry& msg,
                        const OdomConversionOptions& options,
                        const Extrinsic2D& extrinsic = Extrinsic2D::identity()) noexcept;

/// @brief Build a ROS nav_msgs::msg::Odometry message from the current 2D EKF state and covariance.
///
/// Maps the 8D EKF belief into standard 6-DOF pose (x, y, yaw) and body twist (vx, vy, omega)
/// along with the corresponding 6x6 pose and twist covariance blocks (REP-105).
/// @param state Current initialized core::EkfState.
/// @param stamp ROS timestamp to attach to the header.
/// @param odom_frame_id Reference frame ID (e.g., "odom").
/// @param base_frame_id Body child frame ID (e.g., "base_link").
/// @return Populated nav_msgs::msg::Odometry message.
[[nodiscard]] nav_msgs::msg::Odometry stateToOdometryMessage(
    const core::EkfState& state,
    const rclcpp::Time& stamp,
    const std::string& odom_frame_id,
    const std::string& base_frame_id);

/// @brief Build a TF2 TransformStamped (odom_frame_id -> base_frame_id) from the 2D EKF state.
/// @param state Current initialized core::EkfState.
/// @param stamp ROS timestamp to attach to the header.
/// @param odom_frame_id Parent reference frame ID (e.g., "odom").
/// @param base_frame_id Child body frame ID (e.g., "base_link").
/// @return Populated geometry_msgs::msg::TransformStamped message.
[[nodiscard]] geometry_msgs::msg::TransformStamped stateToTransformMessage(
    const core::EkfState& state,
    const rclcpp::Time& stamp,
    const std::string& odom_frame_id,
    const std::string& base_frame_id);

}  // namespace slam_sensor_fusion::ros_node
