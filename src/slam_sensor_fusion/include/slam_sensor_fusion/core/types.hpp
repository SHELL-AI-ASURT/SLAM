#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstddef>

namespace slam_sensor_fusion::core {

/// @brief Dimension of the 2D EKF state vector: [x, y, theta, vx, vy, omega, ax, ay]^T.
/// @see docs/theory/ekf_2d.md, Section 2.
inline constexpr Eigen::Index kStateDim = 8;

/// @brief Fixed-size 8D state vector in SI units.
using StateVector = Eigen::Matrix<double, kStateDim, 1>;

/// @brief Fixed-size 8x8 state covariance or Jacobian matrix.
using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

/// @brief Named indices into the 8D EKF state vector.
/// @see docs/theory/ekf_2d.md, Section 2.
enum class StateIndex : Eigen::Index {
  kX = 0,      ///< Position X in reference frame (odom/map) [m].
  kY = 1,      ///< Position Y in reference frame (odom/map) [m].
  kTheta = 2,  ///< Heading (yaw) in reference frame, normalized to [-pi, pi) [rad].
  kVx = 3,     ///< Longitudinal linear velocity in base_link (+X forward) [m/s].
  kVy = 4,     ///< Lateral linear velocity in base_link (+Y left) [m/s].
  kOmega = 5,  ///< Yaw angular velocity around +Z in base_link [rad/s].
  kAx = 6,     ///< Longitudinal body-velocity derivative dv_x/dt in base_link [m/s^2].
  kAy = 7,     ///< Lateral body-velocity derivative dv_y/dt in base_link [m/s^2].
};

/// @brief Convert a strongly typed StateIndex into an Eigen::Index.
/// @param index State component identifier.
/// @return Underlying zero-based Eigen::Index in [0, kStateDim).
[[nodiscard]] constexpr Eigen::Index toIndex(const StateIndex index) noexcept {
  return static_cast<Eigen::Index>(index);
}

/// @brief Status code returned by Ekf2D prediction operations.
enum class PredictStatus {
  kPredicted,      ///< State and covariance were successfully propagated forward.
  kSkippedZeroDt,  ///< dt == 0.0; state left unchanged without error.
  kNotInitialized, ///< Filter has not been initialized yet.
  kInvalidDt,      ///< dt was non-finite, negative, or exceeded max_dt_sec.
  kInvalidState,   ///< Internal state became non-finite (safeguard triggered).
};

/// @brief Status code returned by Ekf2D measurement update operations.
enum class UpdateStatus {
  kUpdated,            ///< Measurement passed validation and gating and updated the filter.
  kNotInitialized,     ///< Filter has not been initialized yet.
  kNoActiveChannels,   ///< All measurement channels in the struct were disabled.
  kInvalidMeasurement, ///< Measurement values contained NaN/Inf.
  kInvalidCovariance,  ///< Measurement covariance was non-finite, asymmetric, or not SPD.
  kSingularInnovation, ///< Innovation covariance S failed Cholesky LDLT or was near-singular.
  kRejectedOutlier,    ///< Innovation exceeded the configured Mahalanobis chi-squared gate.
};

/// @brief Result of a single EKF measurement update attempt.
struct UpdateResult {
  /// @brief Outcome of the update attempt.
  UpdateStatus status{UpdateStatus::kNotInitialized};
  /// @brief Squared Mahalanobis distance y^T S^{-1} y (0.0 if not computed).
  double mahalanobis_sq{0.0};
  /// @brief Active measurement dimension m in [0, 4].
  Eigen::Index dimension{0};

  /// @brief Convenience predicate indicating the filter state was updated.
  /// @return True iff status == UpdateStatus::kUpdated.
  [[nodiscard]] constexpr bool succeeded() const noexcept {
    return status == UpdateStatus::kUpdated;
  }
};

/// @brief Snapshot of the 2D EKF Gaussian belief N(mean, covariance) at timestamp_sec.
/// @see docs/theory/ekf_2d.md, Section 2.
struct EkfState {
  /// @brief State mean [x, y, theta, vx, vy, omega, ax, ay]^T in SI units.
  StateVector mean{StateVector::Zero()};
  /// @brief Symmetric positive semi-definite 8x8 state error covariance P.
  StateMatrix covariance{StateMatrix::Identity()};
  /// @brief Timestamp corresponding to this state estimate [s].
  double timestamp_sec{0.0};
  /// @brief True once the filter has been initialized with a valid state and covariance.
  bool initialized{false};

  /// @return Reference-frame X position [m].
  [[nodiscard]] double x() const noexcept { return mean(toIndex(StateIndex::kX)); }
  /// @return Reference-frame Y position [m].
  [[nodiscard]] double y() const noexcept { return mean(toIndex(StateIndex::kY)); }
  /// @return Reference-frame heading yaw in [-pi, pi) [rad].
  [[nodiscard]] double theta() const noexcept { return mean(toIndex(StateIndex::kTheta)); }
  /// @return Body-frame longitudinal velocity [m/s].
  [[nodiscard]] double vx() const noexcept { return mean(toIndex(StateIndex::kVx)); }
  /// @return Body-frame lateral velocity [m/s].
  [[nodiscard]] double vy() const noexcept { return mean(toIndex(StateIndex::kVy)); }
  /// @return Body-frame yaw rate [rad/s].
  [[nodiscard]] double omega() const noexcept { return mean(toIndex(StateIndex::kOmega)); }
  /// @return Body-frame longitudinal acceleration dv_x/dt [m/s^2].
  [[nodiscard]] double ax() const noexcept { return mean(toIndex(StateIndex::kAx)); }
  /// @return Body-frame lateral acceleration dv_y/dt [m/s^2].
  [[nodiscard]] double ay() const noexcept { return mean(toIndex(StateIndex::kAy)); }
};

/// @brief Configuration parameters for Ekf2D.
/// @see docs/theory/ekf_2d.md, Section 3.2.
struct EkfConfig {
  /// @brief Positive initial covariance diagonal [x, y, theta, vx, vy, omega, ax, ay] (SI^2).
  StateVector initial_covariance_diag{
      (StateVector() << 0.25, 0.25, 0.10, 0.50, 0.10, 0.10, 1.00, 1.00).finished()};
  /// @brief Non-negative continuous-rate diagonal process noise Q_rate (SI^2 / s).
  StateVector process_noise_rate_diag{
      (StateVector() << 0.02, 0.02, 0.01, 0.50, 0.05, 0.10, 4.00, 2.00).finished()};
  /// @brief Minimum positive dt threshold below which prediction is treated as zero-dt [s].
  double min_dt_sec{1.0e-5};
  /// @brief Maximum allowed single prediction horizon before rejecting as anomalous jump [s].
  double max_dt_sec{1.0};
  /// @brief Maximum integration substep size to bound linearization error [s].
  double max_substep_dt_sec{0.02};
  /// @brief Minimum acceptable LDLT diagonal pivot for innovation covariance S.
  double min_innovation_pivot{1.0e-12};

  /// @brief Validate all configuration bounds and finiteness invariants.
  /// @return True iff all configuration parameters are finite and physically valid.
  [[nodiscard]] bool isValid() const noexcept {
    if (!initial_covariance_diag.allFinite() || !process_noise_rate_diag.allFinite()) {
      return false;
    }
    if ((initial_covariance_diag.array() <= 0.0).any()) {
      return false;
    }
    if ((process_noise_rate_diag.array() < 0.0).any()) {
      return false;
    }
    if (!std::isfinite(min_dt_sec) || !std::isfinite(max_dt_sec) ||
        !std::isfinite(max_substep_dt_sec) || !std::isfinite(min_innovation_pivot)) {
      return false;
    }
    if (min_dt_sec < 0.0 || max_dt_sec <= 0.0 || max_substep_dt_sec <= 0.0 ||
        min_dt_sec > max_dt_sec || min_innovation_pivot <= 0.0) {
      return false;
    }
    return true;
  }
};

/// @brief 2D pose observation [x, y, theta]^T in the reference frame (odom/map).
/// @see docs/theory/ekf_2d.md, Section 3.3.1, Eq. (8).
struct Pose2DMeasurement {
  double x{0.0};      ///< Observed X position in reference frame [m].
  double y{0.0};      ///< Observed Y position in reference frame [m].
  double theta{0.0};  ///< Observed heading yaw in reference frame [rad].
  /// @brief 3x3 symmetric positive definite measurement covariance for [x, y, theta].
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity() * 0.05};
  bool use_position{true};  ///< Fuse (x, y) position channels.
  bool use_yaw{true};       ///< Fuse yaw (theta) channel.
  /// @brief Chi-squared Mahalanobis gate threshold; values <= 0.0 disable gating.
  double mahalanobis_threshold{11.345};
};

/// @brief 2D body-frame velocity observation [vx, vy, omega]^T in base_link (FLU).
/// @see docs/theory/ekf_2d.md, Section 3.3.2, Eq. (9).
struct Twist2DMeasurement {
  double vx{0.0};     ///< Observed longitudinal velocity in base_link [m/s].
  double vy{0.0};     ///< Observed lateral velocity in base_link [m/s].
  double omega{0.0};  ///< Observed yaw angular velocity around +Z [rad/s].
  /// @brief 3x3 symmetric positive definite measurement covariance for [vx, vy, omega].
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity() * 0.05};
  bool use_vx{true};     ///< Fuse longitudinal velocity v_x.
  bool use_vy{true};     ///< Fuse lateral velocity v_y (e.g., non-holonomic vy ~= 0).
  bool use_omega{true};  ///< Fuse yaw rate omega.
  /// @brief Chi-squared Mahalanobis gate threshold; values <= 0.0 disable gating.
  double mahalanobis_threshold{11.345};
};

/// @brief 2D IMU observation [yaw, yaw_rate, ax_imu, ay_imu]^T.
///
/// Linear accelerations (ax, ay) represent horizontal body-frame specific force in base_link
/// with gravity already removed, including the rotating-frame Coriolis terms
/// ax_imu = a_x - omega * v_y and ay_imu = a_y + omega * v_x.
/// @see docs/theory/ekf_2d.md, Section 3.3.3, Eqs. (10)-(12).
struct Imu2DMeasurement {
  double yaw{0.0};       ///< Observed heading yaw in reference frame [rad] (optional).
  double yaw_rate{0.0};  ///< Observed yaw rate around +Z in base_link [rad/s].
  double ax{0.0};        ///< Observed horizontal body specific force along +X [m/s^2].
  double ay{0.0};        ///< Observed horizontal body specific force along +Y [m/s^2].
  /// @brief 4x4 symmetric positive definite measurement covariance for [yaw, yaw_rate, ax, ay].
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity() * 0.1};
  bool use_yaw{false};          ///< Fuse orientation yaw (off by default to avoid magnetometer drift).
  bool use_yaw_rate{true};      ///< Fuse gyroscope yaw rate omega.
  bool use_linear_accel{true};  ///< Fuse horizontal accelerometer specific force (ax, ay).
  /// @brief Chi-squared Mahalanobis gate threshold; values <= 0.0 disable gating.
  double mahalanobis_threshold{13.277};
};

/// @brief Configuration for DifferentialAdapter (consecutive pose-to-twist differentiation).
/// @see docs/theory/ekf_2d.md, Section 3.5, Eqs. (20)-(22).
struct DifferentialAdapterConfig {
  /// @brief Minimum positive time gap between consecutive samples [s].
  double min_dt_sec{1.0e-4};
  /// @brief Maximum time gap before re-anchoring after a sensor dropout [s].
  double max_dt_sec{1.0};
  /// @brief Incremental per-step pose covariance for [dx, dy, dtheta] (SI^2).
  Eigen::Matrix3d default_step_covariance{
      (Eigen::Vector3d(0.004, 0.004, 0.0015)).asDiagonal()};
  /// @brief If true, treat Pose2DMeasurement::covariance as incremental step covariance.
  /// If false (recommended for accumulated odometry), always use default_step_covariance.
  bool use_measurement_covariance{false};
  /// @brief Chi-squared Mahalanobis gate threshold attached to emitted Twist2DMeasurement.
  double mahalanobis_threshold{11.345};

  /// @brief Validate timing bounds and covariance validity.
  /// @return True iff parameters are finite and physically valid.
  [[nodiscard]] bool isValid() const noexcept {
    if (!std::isfinite(min_dt_sec) || !std::isfinite(max_dt_sec) ||
        !std::isfinite(mahalanobis_threshold)) {
      return false;
    }
    if (min_dt_sec <= 0.0 || max_dt_sec <= min_dt_sec) {
      return false;
    }
    if (!default_step_covariance.allFinite()) {
      return false;
    }
    return true;
  }
};

}  // namespace slam_sensor_fusion::core
