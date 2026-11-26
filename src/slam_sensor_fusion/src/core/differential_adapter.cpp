#include "slam_sensor_fusion/core/differential_adapter.hpp"

#include "slam_sensor_fusion/core/math_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace slam_sensor_fusion::core {

DifferentialAdapter::DifferentialAdapter(const DifferentialAdapterConfig& config)
    : config_(config) {
  if (!config_.isValid() || !isSymmetricPositiveDefinite(config_.default_step_covariance)) {
    throw std::invalid_argument("DifferentialAdapter: invalid configuration or step covariance.");
  }
}

void DifferentialAdapter::reset() noexcept {
  has_previous_ = false;
  prev_x_ = 0.0;
  prev_y_ = 0.0;
  prev_theta_ = 0.0;
  prev_timestamp_sec_ = 0.0;
}

DifferentialOutput DifferentialAdapter::process(const Pose2DMeasurement& pose,
                                                const double timestamp_sec) noexcept {
  DifferentialOutput output{};

  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
      !std::isfinite(timestamp_sec)) {
    output.status = DifferentialStatus::kInvalidInput;
    return output;
  }

  const Eigen::Matrix3d* step_cov_ptr = &config_.default_step_covariance;
  if (config_.use_measurement_covariance) {
    if (!isSymmetricPositiveDefinite(pose.covariance)) {
      output.status = DifferentialStatus::kInvalidInput;
      return output;
    }
    step_cov_ptr = &pose.covariance;
  }

  const double wrapped_theta = wrapAngle(pose.theta);

  if (!has_previous_) {
    prev_x_ = pose.x;
    prev_y_ = pose.y;
    prev_theta_ = wrapped_theta;
    prev_timestamp_sec_ = timestamp_sec;
    has_previous_ = true;
    output.status = DifferentialStatus::kInitializedFirstSample;
    return output;
  }

  const double dt_sec = timestamp_sec - prev_timestamp_sec_;
  if (dt_sec <= 0.0) {
    output.status = DifferentialStatus::kStaleTimestamp;
    return output;
  }
  if (dt_sec < config_.min_dt_sec) {
    // Too close in time to differentiate reliably; preserve anchor for the next sample.
    output.status = DifferentialStatus::kInvalidDt;
    return output;
  }
  if (dt_sec > config_.max_dt_sec) {
    // Large gap (e.g., temporary sensor dropout): re-anchor to the new valid pose.
    prev_x_ = pose.x;
    prev_y_ = pose.y;
    prev_theta_ = wrapped_theta;
    prev_timestamp_sec_ = timestamp_sec;
    output.status = DifferentialStatus::kInvalidDt;
    return output;
  }

  // See docs/theory/ekf_2d.md, Section 3.5, Eqs. (20)-(22).
  const double dx_world = pose.x - prev_x_;
  const double dy_world = pose.y - prev_y_;
  const double dtheta = wrapAngle(wrapped_theta - prev_theta_);

  const double theta_mid = wrapAngle(prev_theta_ + 0.5 * dtheta);
  const Eigen::Matrix2d R_body_from_world = rotation2D(theta_mid).transpose();
  const Eigen::Vector2d dp_body = R_body_from_world * Eigen::Vector2d(dx_world, dy_world);

  const double inv_dt = 1.0 / dt_sec;
  const double inv_dt_sq = inv_dt * inv_dt;

  Eigen::Matrix3d T_rot = Eigen::Matrix3d::Identity();
  T_rot.topLeftCorner<2, 2>() = R_body_from_world;

  const Eigen::Matrix3d R_step_sym = 0.5 * (*step_cov_ptr + step_cov_ptr->transpose());
  Eigen::Matrix3d R_twist = inv_dt_sq * (T_rot * R_step_sym * T_rot.transpose());
  R_twist = 0.5 * (R_twist + R_twist.transpose());

  output.twist.vx = dp_body.x() * inv_dt;
  output.twist.vy = dp_body.y() * inv_dt;
  output.twist.omega = dtheta * inv_dt;
  output.twist.covariance = R_twist;
  output.twist.use_vx = pose.use_position;
  output.twist.use_vy = pose.use_position;
  output.twist.use_omega = pose.use_yaw;
  output.twist.mahalanobis_threshold = config_.mahalanobis_threshold;
  output.status = DifferentialStatus::kReady;

  prev_x_ = pose.x;
  prev_y_ = pose.y;
  prev_theta_ = wrapped_theta;
  prev_timestamp_sec_ = timestamp_sec;
  return output;
}

}  // namespace slam_sensor_fusion::core
