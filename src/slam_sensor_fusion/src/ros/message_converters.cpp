#include "slam_sensor_fusion/ros/message_converters.hpp"

#include "slam_sensor_fusion/core/math_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace slam_sensor_fusion::ros_node {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kLargeUnobservedVariance = 1.0e6;

[[nodiscard]] Eigen::Matrix3d extractPlanarCovarianceFromRos6x6(
    const std::array<double, 36>& cov6x6) noexcept {
  // Indices in ROS 6x6 covariance: 0=x, 1=y, 5=yaw (or 0=vx, 1=vy, 5=omega).
  constexpr std::array<std::size_t, 3> kIdx{0, 1, 5};
  Eigen::Matrix3d cov3 = Eigen::Matrix3d::Zero();
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      const std::size_t ros_r = kIdx[static_cast<std::size_t>(r)];
      const std::size_t ros_c = kIdx[static_cast<std::size_t>(c)];
      cov3(r, c) = cov6x6[ros_r * 6U + ros_c];
    }
  }
  return cov3;
}

[[nodiscard]] geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw_rad) noexcept {
  const double half_yaw = 0.5 * yaw_rad;
  geometry_msgs::msg::Quaternion q{};
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

}  // namespace

std::string sanitizeFrameId(const std::string_view frame_id) {
  std::size_t start = 0;
  while (start < frame_id.size() && frame_id[start] == '/') {
    ++start;
  }
  return std::string(frame_id.substr(start));
}

std::optional<RollPitchYaw> extractRollPitchYaw(
    const geometry_msgs::msg::Quaternion& q) noexcept {
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w)) {
    return std::nullopt;
  }
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    return std::nullopt;
  }

  const double inv_norm = 1.0 / norm;
  const double x = q.x * inv_norm;
  const double y = q.y * inv_norm;
  const double z = q.z * inv_norm;
  const double w = q.w * inv_norm;

  // Standard ZYX intrinsic Tait-Bryan extraction from unit quaternion.
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sinp);

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = core::wrapAngle(std::atan2(siny_cosp, cosy_cosp));

  return RollPitchYaw{roll, pitch, yaw};
}

std::optional<Extrinsic2D> extractExtrinsic2D(
    const geometry_msgs::msg::TransformStamped& tf_base_from_sensor) noexcept {
  const auto& t = tf_base_from_sensor.transform.translation;
  if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {
    return std::nullopt;
  }
  const auto rpy = extractRollPitchYaw(tf_base_from_sensor.transform.rotation);
  if (!rpy.has_value()) {
    return std::nullopt;
  }
  return Extrinsic2D{t.x, t.y, rpy->yaw};
}

core::Pose2DMeasurement transformPose2DToBase(const core::Pose2DMeasurement& pose_in_sensor,
                                              const Extrinsic2D& extrinsic) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.6, Eq. (23): T_w_b = T_w_s * T_b_s^{-1}.
  core::Pose2DMeasurement out = pose_in_sensor;
  const double theta_base = core::wrapAngle(pose_in_sensor.theta - extrinsic.yaw);
  const Eigen::Matrix2d R_world_from_base = core::rotation2D(theta_base);
  const Eigen::Vector2d offset_world =
      R_world_from_base * Eigen::Vector2d(extrinsic.tx, extrinsic.ty);

  out.x = pose_in_sensor.x - offset_world.x();
  out.y = pose_in_sensor.y - offset_world.y();
  out.theta = theta_base;

  // First-order Jacobian of [p_s - R(theta_s - psi)*t, theta_s - psi] w.r.t. [p_s, theta_s].
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 2) = offset_world.y();
  J(1, 2) = -offset_world.x();
  Eigen::Matrix3d cov = J * pose_in_sensor.covariance * J.transpose();
  out.covariance = 0.5 * (cov + cov.transpose());
  return out;
}

core::Twist2DMeasurement transformTwist2DToBase(const core::Twist2DMeasurement& twist_in_sensor,
                                                const Extrinsic2D& extrinsic) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.6, Eqs. (24)-(25).
  core::Twist2DMeasurement out = twist_in_sensor;
  const Eigen::Matrix2d R_base_from_sensor = core::rotation2D(extrinsic.yaw);
  const Eigen::Vector2d v_rot =
      R_base_from_sensor * Eigen::Vector2d(twist_in_sensor.vx, twist_in_sensor.vy);

  out.vx = v_rot.x() + twist_in_sensor.omega * extrinsic.ty;
  out.vy = v_rot.y() - twist_in_sensor.omega * extrinsic.tx;
  out.omega = twist_in_sensor.omega;

  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J.topLeftCorner<2, 2>() = R_base_from_sensor;
  J(0, 2) = extrinsic.ty;
  J(1, 2) = -extrinsic.tx;

  Eigen::Matrix3d cov = J * twist_in_sensor.covariance * J.transpose();
  out.covariance = 0.5 * (cov + cov.transpose());
  return out;
}

std::optional<ConvertedMeasurement<core::Imu2DMeasurement>> convertImuMessage(
    const sensor_msgs::msg::Imu& msg,
    const ImuConversionOptions& options,
    const Extrinsic2D& extrinsic) noexcept {
  const auto& ang = msg.angular_velocity;
  const auto& acc = msg.linear_acceleration;
  if (!std::isfinite(ang.z) || !std::isfinite(acc.x) || !std::isfinite(acc.y) ||
      !std::isfinite(acc.z)) {
    return std::nullopt;
  }

  ConvertedMeasurement<core::Imu2DMeasurement> result{};
  auto& out = result.measurement;
  out.use_yaw = options.use_yaw;
  out.use_yaw_rate = options.use_yaw_rate;
  out.use_linear_accel = options.use_linear_accel;
  out.mahalanobis_threshold = options.mahalanobis_threshold;

  // REP-145: orientation_covariance[0] == -1 indicates orientation is unavailable.
  const bool orientation_flagged_unavailable = msg.orientation_covariance[0] < 0.0;
  const auto rpy =
      orientation_flagged_unavailable ? std::nullopt : extractRollPitchYaw(msg.orientation);

  if (out.use_yaw) {
    if (!rpy.has_value()) {
      out.use_yaw = false;
    } else {
      out.yaw = core::wrapAngle(rpy->yaw - extrinsic.yaw);
    }
  }

  out.yaw_rate = ang.z;

  // Horizontal acceleration in sensor frame, optionally removing gravity projection (Eq. 13).
  double ax_sensor = acc.x;
  double ay_sensor = acc.y;
  if (options.compensate_gravity && rpy.has_value()) {
    const double sin_roll = std::sin(rpy->roll);
    const double sin_pitch = std::sin(rpy->pitch);
    const double cos_pitch = std::cos(rpy->pitch);
    const double gx_reaction = -kStandardGravityMps2 * sin_pitch;
    const double gy_reaction = kStandardGravityMps2 * sin_roll * cos_pitch;
    ax_sensor -= gx_reaction;
    ay_sensor -= gy_reaction;
  }

  // Rotate horizontal specific force into base_link and compensate centripetal lever arm.
  const Eigen::Matrix2d R_base_from_sensor = core::rotation2D(extrinsic.yaw);
  const Eigen::Vector2d a_base =
      R_base_from_sensor * Eigen::Vector2d(ax_sensor, ay_sensor) +
      (ang.z * ang.z) * Eigen::Vector2d(extrinsic.tx, extrinsic.ty);
  out.ax = a_base.x();
  out.ay = a_base.y();

  // Populate 4x4 covariance for [yaw, yaw_rate, ax, ay], falling back per block if zero/invalid.
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  bool fallback_used = false;

  // Block 0: yaw variance from orientation_covariance(2, 2) = index 8.
  const double yaw_var_msg = msg.orientation_covariance[8];
  if (options.use_message_covariance && std::isfinite(yaw_var_msg) && yaw_var_msg > 1.0e-12) {
    cov(0, 0) = yaw_var_msg;
  } else {
    cov(0, 0) = options.fallback_yaw_variance;
    if (out.use_yaw) {
      fallback_used = true;
    }
  }

  // Block 1: yaw rate variance from angular_velocity_covariance(2, 2) = index 8.
  const double wz_var_msg = msg.angular_velocity_covariance[8];
  if (options.use_message_covariance && std::isfinite(wz_var_msg) && wz_var_msg > 1.0e-12) {
    cov(1, 1) = wz_var_msg;
  } else {
    cov(1, 1) = options.fallback_yaw_rate_variance;
    if (out.use_yaw_rate) {
      fallback_used = true;
    }
  }

  // Block 2-3: horizontal linear acceleration 2x2 covariance rotated to base_link.
  Eigen::Matrix2d acc_cov_sensor;
  acc_cov_sensor << msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[1],
      msg.linear_acceleration_covariance[3], msg.linear_acceleration_covariance[4];
  if (options.use_message_covariance && core::isSymmetricPositiveDefinite(acc_cov_sensor)) {
    const Eigen::Matrix2d acc_cov_base =
        R_base_from_sensor * acc_cov_sensor * R_base_from_sensor.transpose();
    cov.bottomRightCorner<2, 2>() = 0.5 * (acc_cov_base + acc_cov_base.transpose());
  } else {
    cov(2, 2) = options.fallback_accel_variance.x();
    cov(3, 3) = options.fallback_accel_variance.y();
    if (out.use_linear_accel) {
      fallback_used = true;
    }
  }

  out.covariance = cov;
  result.used_fallback_covariance = fallback_used;
  return result;
}

std::optional<ConvertedMeasurement<core::Pose2DMeasurement>> convertOdomPoseMessage(
    const nav_msgs::msg::Odometry& msg,
    const OdomConversionOptions& options,
    const Extrinsic2D& extrinsic) noexcept {
  const auto& pos = msg.pose.pose.position;
  if (!std::isfinite(pos.x) || !std::isfinite(pos.y)) {
    return std::nullopt;
  }
  const auto rpy = extractRollPitchYaw(msg.pose.pose.orientation);
  if (!rpy.has_value()) {
    return std::nullopt;
  }

  core::Pose2DMeasurement pose_sensor{};
  pose_sensor.x = pos.x;
  pose_sensor.y = pos.y;
  pose_sensor.theta = rpy->yaw;
  pose_sensor.use_position = options.use_position;
  pose_sensor.use_yaw = options.use_yaw;
  pose_sensor.mahalanobis_threshold = options.mahalanobis_threshold;

  bool fallback_used = false;
  const Eigen::Matrix3d cov_msg = extractPlanarCovarianceFromRos6x6(msg.pose.covariance);
  if (options.use_message_covariance && core::isSymmetricPositiveDefinite(cov_msg)) {
    pose_sensor.covariance = 0.5 * (cov_msg + cov_msg.transpose());
  } else {
    pose_sensor.covariance = options.fallback_pose_variance.asDiagonal();
    fallback_used = true;
  }

  ConvertedMeasurement<core::Pose2DMeasurement> result{};
  result.measurement = transformPose2DToBase(pose_sensor, extrinsic);
  result.used_fallback_covariance = fallback_used;
  return result;
}

std::optional<ConvertedMeasurement<core::Twist2DMeasurement>> convertOdomTwistMessage(
    const nav_msgs::msg::Odometry& msg,
    const OdomConversionOptions& options,
    const Extrinsic2D& extrinsic) noexcept {
  const auto& lin = msg.twist.twist.linear;
  const auto& ang = msg.twist.twist.angular;
  if (!std::isfinite(lin.x) || !std::isfinite(lin.y) || !std::isfinite(ang.z)) {
    return std::nullopt;
  }

  core::Twist2DMeasurement twist_sensor{};
  twist_sensor.vx = lin.x;
  twist_sensor.vy = lin.y;
  twist_sensor.omega = ang.z;
  twist_sensor.use_vx = options.use_vx;
  twist_sensor.use_vy = options.use_vy;
  twist_sensor.use_omega = options.use_omega;
  twist_sensor.mahalanobis_threshold = options.mahalanobis_threshold;

  bool fallback_used = false;
  const Eigen::Matrix3d cov_msg = extractPlanarCovarianceFromRos6x6(msg.twist.covariance);
  if (options.use_message_covariance && core::isSymmetricPositiveDefinite(cov_msg)) {
    twist_sensor.covariance = 0.5 * (cov_msg + cov_msg.transpose());
  } else {
    twist_sensor.covariance = options.fallback_twist_variance.asDiagonal();
    fallback_used = true;
  }

  ConvertedMeasurement<core::Twist2DMeasurement> result{};
  result.measurement = transformTwist2DToBase(twist_sensor, extrinsic);
  result.used_fallback_covariance = fallback_used;
  return result;
}

nav_msgs::msg::Odometry stateToOdometryMessage(const core::EkfState& state,
                                               const rclcpp::Time& stamp,
                                               const std::string& odom_frame_id,
                                               const std::string& base_frame_id) {
  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = base_frame_id;

  odom.pose.pose.position.x = state.x();
  odom.pose.pose.position.y = state.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yawToQuaternion(state.theta());

  odom.twist.twist.linear.x = state.vx();
  odom.twist.twist.linear.y = state.vy();
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = state.omega();

  odom.pose.covariance.fill(0.0);
  odom.twist.covariance.fill(0.0);

  // Map 2D EKF pose covariance [x, y, theta] -> ROS 6x6 indices [0, 1, 5].
  constexpr std::array<std::size_t, 3> kRosIdx{0, 1, 5};
  constexpr std::array<core::StateIndex, 3> kPoseStateIdx{
      core::StateIndex::kX, core::StateIndex::kY, core::StateIndex::kTheta};
  constexpr std::array<core::StateIndex, 3> kTwistStateIdx{
      core::StateIndex::kVx, core::StateIndex::kVy, core::StateIndex::kOmega};

  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      const std::size_t flat = kRosIdx[r] * 6U + kRosIdx[c];
      odom.pose.covariance[flat] = state.covariance(
          core::toIndex(kPoseStateIdx[r]), core::toIndex(kPoseStateIdx[c]));
      odom.twist.covariance[flat] = state.covariance(
          core::toIndex(kTwistStateIdx[r]), core::toIndex(kTwistStateIdx[c]));
    }
  }

  // Mark unobserved 3D axes (z, roll, pitch and vz, wx, wy) with large diagonal variance.
  for (const std::size_t idx : {std::size_t{2}, std::size_t{3}, std::size_t{4}}) {
    odom.pose.covariance[idx * 6U + idx] = kLargeUnobservedVariance;
    odom.twist.covariance[idx * 6U + idx] = kLargeUnobservedVariance;
  }

  return odom;
}

geometry_msgs::msg::TransformStamped stateToTransformMessage(const core::EkfState& state,
                                                             const rclcpp::Time& stamp,
                                                             const std::string& odom_frame_id,
                                                             const std::string& base_frame_id) {
  geometry_msgs::msg::TransformStamped tf_msg{};
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = odom_frame_id;
  tf_msg.child_frame_id = base_frame_id;
  tf_msg.transform.translation.x = state.x();
  tf_msg.transform.translation.y = state.y();
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = yawToQuaternion(state.theta());
  return tf_msg;
}

}  // namespace slam_sensor_fusion::ros_node
