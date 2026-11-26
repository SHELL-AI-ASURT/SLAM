#pragma once

#include "slam_sensor_fusion/core/types.hpp"

#include <Eigen/Core>

#include <array>

namespace slam_sensor_fusion::core {

/// @brief Pure C++20 / Eigen 2D Extended Kalman Filter for planar vehicle sensor fusion.
///
/// Estimates the 8D state x = [x, y, theta, vx, vy, omega, ax, ay]^T in SI units using
/// midpoint-heading kinematics for prediction and LDLT + Mahalanobis-gated Joseph-form
/// updates for pose, body-frame twist, and Coriolis-aware IMU measurements.
///
/// Thread safety: not internally synchronized; callers sharing an instance across threads
/// must serialize access externally.
/// @see docs/theory/ekf_2d.md.
class Ekf2D final {
 public:
  /// @brief Construct an uninitialized Ekf2D with validated configuration.
  /// @param config Filter noise, timing, and numerical stability parameters.
  /// @throws std::invalid_argument If config.isValid() is false.
  explicit Ekf2D(const EkfConfig& config);

  /// @brief Initialize the filter belief with an explicit mean, covariance, and timestamp.
  ///
  /// Preconditions: initial_mean and timestamp_sec must be finite; initial_covariance must be
  /// symmetric and positive definite.
  /// Postconditions: On success, isInitialized() is true and state().theta() is in [-pi, pi).
  /// On failure, previous state is left unchanged.
  /// @param initial_mean Initial 8D state vector in SI units.
  /// @param initial_covariance Initial symmetric positive definite 8x8 covariance matrix.
  /// @param timestamp_sec Initial timestamp [s].
  /// @return True if initialization succeeded; false if inputs failed validation.
  [[nodiscard]] bool initialize(const StateVector& initial_mean,
                                const StateMatrix& initial_covariance,
                                double timestamp_sec) noexcept;

  /// @brief Initialize the filter at zero state with configured initial_covariance_diag.
  /// @param timestamp_sec Initial finite timestamp [s].
  /// @return True if timestamp_sec is finite and initialization succeeded.
  [[nodiscard]] bool initializeAtOrigin(double timestamp_sec) noexcept;

  /// @brief Reset the filter back to an uninitialized state.
  void reset() noexcept;

  /// @brief Propagate state and covariance forward by dt_sec using midpoint-heading kinematics.
  ///
  /// Subdivides intervals exceeding config.max_substep_dt_sec into uniform substeps to limit
  /// linearization error. Leaves state untouched if dt_sec is negative, non-finite, or exceeds
  /// config.max_dt_sec.
  /// @param dt_sec Non-negative elapsed time [s].
  /// @return PredictStatus describing whether prediction ran, skipped, or failed validation.
  /// @see docs/theory/ekf_2d.md, Section 3.2, Eqs. (2)-(7).
  [[nodiscard]] PredictStatus predict(double dt_sec) noexcept;

  /// @brief Propagate state and covariance from state().timestamp_sec to target_timestamp_sec.
  /// @param target_timestamp_sec Monotonically non-decreasing target timestamp [s].
  /// @return PredictStatus (kInvalidDt if target_timestamp_sec < current timestamp).
  [[nodiscard]] PredictStatus predictTo(double target_timestamp_sec) noexcept;

  /// @brief Fuse a 2D pose observation [x, y, theta]^T in the reference frame.
  /// @param meas Pose observation, enabled channel flags, covariance, and Mahalanobis gate.
  /// @return UpdateResult containing status, squared Mahalanobis distance, and active dimension.
  /// @see docs/theory/ekf_2d.md, Section 3.3.1 & Section 3.4, Eqs. (8), (14)-(19).
  [[nodiscard]] UpdateResult updatePose(const Pose2DMeasurement& meas) noexcept;

  /// @brief Fuse a 2D body-frame velocity observation [vx, vy, omega]^T in base_link.
  /// @param meas Twist observation, enabled channel flags, covariance, and Mahalanobis gate.
  /// @return UpdateResult containing status, squared Mahalanobis distance, and active dimension.
  /// @see docs/theory/ekf_2d.md, Section 3.3.2 & Section 3.4, Eqs. (9), (14)-(19).
  [[nodiscard]] UpdateResult updateTwist(const Twist2DMeasurement& meas) noexcept;

  /// @brief Fuse a 2D IMU observation [yaw, yaw_rate, ax_imu, ay_imu]^T with Coriolis physics.
  /// @param meas IMU observation, enabled channel flags, covariance, and Mahalanobis gate.
  /// @return UpdateResult containing status, squared Mahalanobis distance, and active dimension.
  /// @see docs/theory/ekf_2d.md, Section 3.3.3 & Section 3.4, Eqs. (10)-(12), (14)-(19).
  [[nodiscard]] UpdateResult updateImu(const Imu2DMeasurement& meas) noexcept;

  /// @return Const reference to the current filter state belief.
  [[nodiscard]] const EkfState& state() const noexcept { return state_; }

  /// @return Const reference to the active filter configuration.
  [[nodiscard]] const EkfConfig& config() const noexcept { return config_; }

  /// @return True iff the filter has been initialized.
  [[nodiscard]] bool isInitialized() const noexcept { return state_.initialized; }

  /// @brief Evaluate the nonlinear single-step kinematic transition x^- = f(x, dt).
  ///
  /// Pure static function exposed for analytical testing and finite-difference Jacobian checks.
  /// @param x Current 8D state vector.
  /// @param dt_sec Positive time interval [s].
  /// @return Propagated 8D state vector with wrapped heading in [-pi, pi).
  /// @see docs/theory/ekf_2d.md, Section 3.2, Eqs. (2)-(4).
  [[nodiscard]] static StateVector propagateState(const StateVector& x, double dt_sec) noexcept;

  /// @brief Compute the analytical 8x8 process Jacobian F = df/dx at (x, dt).
  /// @param x State vector at which to linearize.
  /// @param dt_sec Time step [s].
  /// @return Analytical 8x8 Jacobian matrix F.
  /// @see docs/theory/ekf_2d.md, Section 3.2, Eqs. (5)-(6).
  [[nodiscard]] static StateMatrix computeProcessJacobian(const StateVector& x,
                                                          double dt_sec) noexcept;

  /// @brief Predict the full 4D IMU measurement h_imu(x) = [theta, omega, ax - w*vy, ay + w*vx]^T.
  /// @param x Current 8D state vector.
  /// @return Predicted 4D IMU observation in SI units.
  /// @see docs/theory/ekf_2d.md, Section 3.3.3, Eqs. (10)-(11).
  [[nodiscard]] static Eigen::Vector4d predictImuMeasurement(const StateVector& x) noexcept;

  /// @brief Compute the analytical 4x8 IMU measurement Jacobian H_imu = dh_imu/dx.
  /// @param x State vector at which to linearize.
  /// @return Analytical 4x8 Jacobian matrix H_imu.
  /// @see docs/theory/ekf_2d.md, Section 3.3.3, Eq. (12).
  [[nodiscard]] static Eigen::Matrix<double, 4, kStateDim> computeImuJacobian(
      const StateVector& x) noexcept;

 private:
  void predictSubstep(double dt_sec) noexcept;

  template <int MeasDim>
  [[nodiscard]] UpdateResult applyUpdateFixed(
      const Eigen::Matrix<double, MeasDim, 1>& z,
      const Eigen::Matrix<double, MeasDim, 1>& z_hat,
      const Eigen::Matrix<double, MeasDim, kStateDim>& H,
      const Eigen::Matrix<double, MeasDim, MeasDim>& R,
      const std::array<bool, static_cast<std::size_t>(MeasDim)>& angular_mask,
      double mahalanobis_threshold) noexcept;

  template <int Dim>
  [[nodiscard]] UpdateResult updateTwistFixed(
      const Twist2DMeasurement& meas,
      const std::array<Eigen::Index, 3>& active_local,
      const std::array<StateIndex, 3>& active_state) noexcept;

  template <int Dim>
  [[nodiscard]] UpdateResult updateImuFixed(
      const Imu2DMeasurement& meas,
      const std::array<Eigen::Index, 4>& active_rows) noexcept;

  EkfConfig config_;
  EkfState state_;
};

}  // namespace slam_sensor_fusion::core
