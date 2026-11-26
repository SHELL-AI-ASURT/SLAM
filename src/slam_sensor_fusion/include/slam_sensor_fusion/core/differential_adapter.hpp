#pragma once

#include "slam_sensor_fusion/core/types.hpp"

namespace slam_sensor_fusion::core {

/// @brief Outcome of processing an accumulated 2D pose through DifferentialAdapter.
enum class DifferentialStatus {
  kReady,                   ///< Valid body-frame twist computed from consecutive poses.
  kInitializedFirstSample,  ///< First valid sample anchored; no twist emitted yet.
  kInvalidInput,            ///< Pose values, covariance, or timestamp were non-finite/invalid.
  kStaleTimestamp,          ///< Timestamp <= previous sample timestamp; sample ignored.
  kInvalidDt,               ///< dt fell outside [min_dt_sec, max_dt_sec]; re-anchored if dt > max.
};

/// @brief Output of DifferentialAdapter::process().
struct DifferentialOutput {
  /// @brief Processing outcome status.
  DifferentialStatus status{DifferentialStatus::kInvalidInput};
  /// @brief Computed body-frame twist measurement (valid only when status == kReady).
  Twist2DMeasurement twist{};

  /// @return True iff twist is valid and ready to fuse into Ekf2D::updateTwist().
  [[nodiscard]] constexpr bool hasTwist() const noexcept {
    return status == DifferentialStatus::kReady;
  }
};

/// @brief Converts consecutive accumulated 2D odometry poses into body-frame twist measurements.
///
/// Prevents drift fighting when fusing multiple independent drifting odometry nodes (e.g.,
/// wheel odometry, scan-to-scan LiDAR matching, and visual odometry) into one EKF.
/// Uses midpoint heading rotation for second-order geometric accuracy during turns.
///
/// Thread safety: not internally synchronized; one instance should be owned per odometry source.
/// @see docs/theory/ekf_2d.md, Section 3.5, Eqs. (20)-(22).
class DifferentialAdapter final {
 public:
  /// @brief Construct a DifferentialAdapter with validated timing and noise configuration.
  /// @param config Timing bounds and incremental step covariance parameters.
  /// @throws std::invalid_argument If config.isValid() is false or default_step_covariance is
  /// not symmetric positive definite.
  explicit DifferentialAdapter(const DifferentialAdapterConfig& config);

  /// @brief Process a new 2D pose sample and produce a body-frame twist when possible.
  ///
  /// Edge-case behavior:
  /// - Non-finite inputs or invalid covariance leave the stored anchor unchanged.
  /// - Non-increasing timestamps (dt <= 0) return kStaleTimestamp without altering the anchor.
  /// - Very small dt (< min_dt_sec) returns kInvalidDt without advancing the anchor.
  /// - Very large dt (> max_dt_sec, e.g., after dropout) re-anchors to the current valid pose
  ///   and returns kInvalidDt so subsequent samples resume cleanly.
  /// @param pose Accumulated 2D pose in the sensor's reference frame [m, m, rad].
  /// @param timestamp_sec Finite timestamp of the pose [s].
  /// @return DifferentialOutput containing status and (when kReady) body-frame Twist2DMeasurement.
  /// @see docs/theory/ekf_2d.md, Section 3.5, Eqs. (20)-(22).
  [[nodiscard]] DifferentialOutput process(const Pose2DMeasurement& pose,
                                           double timestamp_sec) noexcept;

  /// @brief Clear the stored previous pose anchor (e.g., on simulation clock reset).
  void reset() noexcept;

  /// @return True iff a previous pose anchor is currently stored.
  [[nodiscard]] bool hasPrevious() const noexcept { return has_previous_; }

  /// @return Const reference to the adapter configuration.
  [[nodiscard]] const DifferentialAdapterConfig& config() const noexcept { return config_; }

 private:
  DifferentialAdapterConfig config_;
  bool has_previous_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_theta_{0.0};
  double prev_timestamp_sec_{0.0};
};

}  // namespace slam_sensor_fusion::core
