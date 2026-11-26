#include "slam_sensor_fusion/core/ekf_2d.hpp"

#include "slam_sensor_fusion/core/math_utils.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace slam_sensor_fusion::core {

Ekf2D::Ekf2D(const EkfConfig& config) : config_(config) {
  if (!config_.isValid()) {
    throw std::invalid_argument("Ekf2D: invalid EkfConfig parameters.");
  }
}

bool Ekf2D::initialize(const StateVector& initial_mean,
                       const StateMatrix& initial_covariance,
                       const double timestamp_sec) noexcept {
  if (!initial_mean.allFinite() || !std::isfinite(timestamp_sec)) {
    return false;
  }
  if (!isSymmetricPositiveDefinite(initial_covariance)) {
    return false;
  }

  state_.mean = initial_mean;
  state_.mean(toIndex(StateIndex::kTheta)) = wrapAngle(state_.mean(toIndex(StateIndex::kTheta)));
  state_.covariance = 0.5 * (initial_covariance + initial_covariance.transpose());
  state_.timestamp_sec = timestamp_sec;
  state_.initialized = true;
  return true;
}

bool Ekf2D::initializeAtOrigin(const double timestamp_sec) noexcept {
  const StateMatrix init_cov = config_.initial_covariance_diag.asDiagonal();
  return initialize(StateVector::Zero(), init_cov, timestamp_sec);
}

void Ekf2D::reset() noexcept {
  state_ = EkfState{};
}

StateVector Ekf2D::propagateState(const StateVector& x, const double dt_sec) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.2, Eqs. (2)-(4).
  const double px = x(toIndex(StateIndex::kX));
  const double py = x(toIndex(StateIndex::kY));
  const double theta = x(toIndex(StateIndex::kTheta));
  const double vx = x(toIndex(StateIndex::kVx));
  const double vy = x(toIndex(StateIndex::kVy));
  const double omega = x(toIndex(StateIndex::kOmega));
  const double ax = x(toIndex(StateIndex::kAx));
  const double ay = x(toIndex(StateIndex::kAy));

  const double half_dt_sq = 0.5 * dt_sec * dt_sec;
  const double dp_x = vx * dt_sec + ax * half_dt_sq;
  const double dp_y = vy * dt_sec + ay * half_dt_sq;

  // Midpoint heading improves accuracy on curved trajectories without omega==0 singularity.
  const double theta_mid = theta + 0.5 * omega * dt_sec;
  const double c_mid = std::cos(theta_mid);
  const double s_mid = std::sin(theta_mid);

  StateVector next = x;
  next(toIndex(StateIndex::kX)) = px + dp_x * c_mid - dp_y * s_mid;
  next(toIndex(StateIndex::kY)) = py + dp_x * s_mid + dp_y * c_mid;
  next(toIndex(StateIndex::kTheta)) = wrapAngle(theta + omega * dt_sec);
  next(toIndex(StateIndex::kVx)) = vx + ax * dt_sec;
  next(toIndex(StateIndex::kVy)) = vy + ay * dt_sec;
  // omega, ax, ay persist under constant-turn-rate, constant-acceleration kinematics.
  return next;
}

StateMatrix Ekf2D::computeProcessJacobian(const StateVector& x, const double dt_sec) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.2, Eqs. (5)-(6).
  const double theta = x(toIndex(StateIndex::kTheta));
  const double vx = x(toIndex(StateIndex::kVx));
  const double vy = x(toIndex(StateIndex::kVy));
  const double omega = x(toIndex(StateIndex::kOmega));
  const double ax = x(toIndex(StateIndex::kAx));
  const double ay = x(toIndex(StateIndex::kAy));

  const double half_dt = 0.5 * dt_sec;
  const double half_dt_sq = 0.5 * dt_sec * dt_sec;
  const double dp_x = vx * dt_sec + ax * half_dt_sq;
  const double dp_y = vy * dt_sec + ay * half_dt_sq;

  const double theta_mid = theta + half_dt * omega;
  const double c_mid = std::cos(theta_mid);
  const double s_mid = std::sin(theta_mid);

  const double d_x = -dp_x * s_mid - dp_y * c_mid;
  const double d_y = dp_x * c_mid - dp_y * s_mid;

  StateMatrix F = StateMatrix::Identity();
  constexpr Eigen::Index kIx = toIndex(StateIndex::kX);
  constexpr Eigen::Index kIy = toIndex(StateIndex::kY);
  constexpr Eigen::Index kItheta = toIndex(StateIndex::kTheta);
  constexpr Eigen::Index kIvx = toIndex(StateIndex::kVx);
  constexpr Eigen::Index kIvy = toIndex(StateIndex::kVy);
  constexpr Eigen::Index kIomega = toIndex(StateIndex::kOmega);
  constexpr Eigen::Index kIax = toIndex(StateIndex::kAx);
  constexpr Eigen::Index kIay = toIndex(StateIndex::kAy);

  // Row 0: dx'/d(.)
  F(kIx, kItheta) = d_x;
  F(kIx, kIvx) = dt_sec * c_mid;
  F(kIx, kIvy) = -dt_sec * s_mid;
  F(kIx, kIomega) = half_dt * d_x;
  F(kIx, kIax) = half_dt_sq * c_mid;
  F(kIx, kIay) = -half_dt_sq * s_mid;

  // Row 1: dy'/d(.)
  F(kIy, kItheta) = d_y;
  F(kIy, kIvx) = dt_sec * s_mid;
  F(kIy, kIvy) = dt_sec * c_mid;
  F(kIy, kIomega) = half_dt * d_y;
  F(kIy, kIax) = half_dt_sq * s_mid;
  F(kIy, kIay) = half_dt_sq * c_mid;

  // Row 2: dtheta'/d(.)
  F(kItheta, kIomega) = dt_sec;

  // Rows 3-4: dv'/d(.)
  F(kIvx, kIax) = dt_sec;
  F(kIvy, kIay) = dt_sec;

  return F;
}

Eigen::Vector4d Ekf2D::predictImuMeasurement(const StateVector& x) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.3.3, Eqs. (10)-(11).
  const double theta = x(toIndex(StateIndex::kTheta));
  const double vx = x(toIndex(StateIndex::kVx));
  const double vy = x(toIndex(StateIndex::kVy));
  const double omega = x(toIndex(StateIndex::kOmega));
  const double ax = x(toIndex(StateIndex::kAx));
  const double ay = x(toIndex(StateIndex::kAy));

  Eigen::Vector4d h;
  h(0) = theta;
  h(1) = omega;
  h(2) = ax - omega * vy;
  h(3) = ay + omega * vx;
  return h;
}

Eigen::Matrix<double, 4, kStateDim> Ekf2D::computeImuJacobian(const StateVector& x) noexcept {
  // See docs/theory/ekf_2d.md, Section 3.3.3, Eq. (12).
  const double vx = x(toIndex(StateIndex::kVx));
  const double vy = x(toIndex(StateIndex::kVy));
  const double omega = x(toIndex(StateIndex::kOmega));

  Eigen::Matrix<double, 4, kStateDim> H = Eigen::Matrix<double, 4, kStateDim>::Zero();
  H(0, toIndex(StateIndex::kTheta)) = 1.0;
  H(1, toIndex(StateIndex::kOmega)) = 1.0;

  // dh_ax / d(.) where h_ax = ax - omega * vy
  H(2, toIndex(StateIndex::kVy)) = -omega;
  H(2, toIndex(StateIndex::kOmega)) = -vy;
  H(2, toIndex(StateIndex::kAx)) = 1.0;

  // dh_ay / d(.) where h_ay = ay + omega * vx
  H(3, toIndex(StateIndex::kVx)) = omega;
  H(3, toIndex(StateIndex::kOmega)) = vx;
  H(3, toIndex(StateIndex::kAy)) = 1.0;
  return H;
}

void Ekf2D::predictSubstep(const double dt_sec) noexcept {
  const StateMatrix F = computeProcessJacobian(state_.mean, dt_sec);
  state_.mean = propagateState(state_.mean, dt_sec);

  // See docs/theory/ekf_2d.md, Section 3.2, Eq. (7).
  const StateMatrix Q = (config_.process_noise_rate_diag * dt_sec).asDiagonal();
  StateMatrix P_next = F * state_.covariance * F.transpose() + Q;
  state_.covariance = 0.5 * (P_next + P_next.transpose());
}

PredictStatus Ekf2D::predict(const double dt_sec) noexcept {
  if (!state_.initialized) {
    return PredictStatus::kNotInitialized;
  }
  if (!std::isfinite(dt_sec) || dt_sec < 0.0 || dt_sec > config_.max_dt_sec) {
    return PredictStatus::kInvalidDt;
  }
  if (!state_.mean.allFinite() || !state_.covariance.allFinite()) {
    return PredictStatus::kInvalidState;
  }
  if (dt_sec < config_.min_dt_sec) {
    state_.timestamp_sec += dt_sec;
    return PredictStatus::kSkippedZeroDt;
  }

  const int num_substeps =
      static_cast<int>(std::ceil(dt_sec / config_.max_substep_dt_sec));
  const int safe_substeps = num_substeps < 1 ? 1 : num_substeps;
  const double substep_dt = dt_sec / static_cast<double>(safe_substeps);

  for (int i = 0; i < safe_substeps; ++i) {
    predictSubstep(substep_dt);
  }
  state_.timestamp_sec += dt_sec;

  if (!state_.mean.allFinite() || !state_.covariance.allFinite()) {
    return PredictStatus::kInvalidState;
  }
  return PredictStatus::kPredicted;
}

PredictStatus Ekf2D::predictTo(const double target_timestamp_sec) noexcept {
  if (!state_.initialized) {
    return PredictStatus::kNotInitialized;
  }
  if (!std::isfinite(target_timestamp_sec)) {
    return PredictStatus::kInvalidDt;
  }
  const double dt_sec = target_timestamp_sec - state_.timestamp_sec;
  return predict(dt_sec);
}

template <int MeasDim>
UpdateResult Ekf2D::applyUpdateFixed(
    const Eigen::Matrix<double, MeasDim, 1>& z,
    const Eigen::Matrix<double, MeasDim, 1>& z_hat,
    const Eigen::Matrix<double, MeasDim, kStateDim>& H,
    const Eigen::Matrix<double, MeasDim, MeasDim>& R,
    const std::array<bool, static_cast<std::size_t>(MeasDim)>& angular_mask,
    const double mahalanobis_threshold) noexcept {
  UpdateResult result{};
  result.dimension = MeasDim;

  if (!state_.initialized) {
    result.status = UpdateStatus::kNotInitialized;
    return result;
  }
  if (!z.allFinite() || !z_hat.allFinite() || !H.allFinite() ||
      !std::isfinite(mahalanobis_threshold)) {
    result.status = UpdateStatus::kInvalidMeasurement;
    return result;
  }
  if (!isSymmetricPositiveDefinite(R)) {
    result.status = UpdateStatus::kInvalidCovariance;
    return result;
  }

  // Eq. (14): Innovation with manifold wrapping on angular components.
  Eigen::Matrix<double, MeasDim, 1> innovation = z - z_hat;
  for (Eigen::Index i = 0; i < MeasDim; ++i) {
    if (angular_mask[static_cast<std::size_t>(i)]) {
      innovation(i) = wrapAngle(innovation(i));
    }
  }

  // Eq. (15): Innovation covariance S = H P H^T + R, symmetrized.
  const Eigen::Matrix<double, MeasDim, kStateDim> HP = H * state_.covariance;
  Eigen::Matrix<double, MeasDim, MeasDim> S = HP * H.transpose() + R;
  S = 0.5 * (S + S.transpose());

  // Factorize S via Cholesky LDLT (never explicit matrix inversion).
  const Eigen::LDLT<Eigen::Matrix<double, MeasDim, MeasDim>> ldlt(S);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
      (ldlt.vectorD().array() <= config_.min_innovation_pivot).any()) {
    result.status = UpdateStatus::kSingularInnovation;
    return result;
  }

  // Eq. (16): Mahalanobis distance outlier gating.
  const Eigen::Matrix<double, MeasDim, 1> S_inv_y = ldlt.solve(innovation);
  const double mahalanobis_sq = innovation.dot(S_inv_y);
  if (!std::isfinite(mahalanobis_sq) || mahalanobis_sq < -1.0e-12) {
    result.status = UpdateStatus::kSingularInnovation;
    return result;
  }
  result.mahalanobis_sq = mahalanobis_sq < 0.0 ? 0.0 : mahalanobis_sq;

  if (mahalanobis_threshold > 0.0 && result.mahalanobis_sq > mahalanobis_threshold) {
    result.status = UpdateStatus::kRejectedOutlier;
    return result;
  }

  // Eq. (17): Kalman gain K = P H^T S^{-1} = (S^{-1} (H P))^T via LDLT solve.
  const Eigen::Matrix<double, kStateDim, MeasDim> K = ldlt.solve(HP).transpose();
  if (!K.allFinite()) {
    result.status = UpdateStatus::kSingularInnovation;
    return result;
  }

  // Eq. (18): State update with heading wrapped back to [-pi, pi).
  StateVector x_next = state_.mean + K * innovation;
  x_next(toIndex(StateIndex::kTheta)) = wrapAngle(x_next(toIndex(StateIndex::kTheta)));

  // Eq. (19): Joseph-form covariance update guarantees symmetry and positive semi-definiteness.
  const StateMatrix A = StateMatrix::Identity() - K * H;
  const Eigen::Matrix<double, MeasDim, MeasDim> R_sym = 0.5 * (R + R.transpose());
  StateMatrix P_next = A * state_.covariance * A.transpose() + K * R_sym * K.transpose();
  P_next = 0.5 * (P_next + P_next.transpose());

  if (!x_next.allFinite() || !P_next.allFinite()) {
    result.status = UpdateStatus::kSingularInnovation;
    return result;
  }

  state_.mean = x_next;
  state_.covariance = P_next;
  result.status = UpdateStatus::kUpdated;
  return result;
}

UpdateResult Ekf2D::updatePose(const Pose2DMeasurement& meas) noexcept {
  if (!state_.initialized) {
    return UpdateResult{UpdateStatus::kNotInitialized, 0.0, 0};
  }
  if (!meas.use_position && !meas.use_yaw) {
    return UpdateResult{UpdateStatus::kNoActiveChannels, 0.0, 0};
  }

  // See docs/theory/ekf_2d.md, Section 3.3.1, Eq. (8).
  if (meas.use_position && meas.use_yaw) {
    Eigen::Vector3d z(meas.x, meas.y, meas.theta);
    Eigen::Vector3d z_hat(state_.x(), state_.y(), state_.theta());
    Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
    H(0, toIndex(StateIndex::kX)) = 1.0;
    H(1, toIndex(StateIndex::kY)) = 1.0;
    H(2, toIndex(StateIndex::kTheta)) = 1.0;
    return applyUpdateFixed<3>(z, z_hat, H, meas.covariance, {false, false, true},
                               meas.mahalanobis_threshold);
  }

  if (meas.use_position) {
    Eigen::Vector2d z(meas.x, meas.y);
    Eigen::Vector2d z_hat(state_.x(), state_.y());
    Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
    H(0, toIndex(StateIndex::kX)) = 1.0;
    H(1, toIndex(StateIndex::kY)) = 1.0;
    const Eigen::Matrix2d R = meas.covariance.topLeftCorner<2, 2>();
    return applyUpdateFixed<2>(z, z_hat, H, R, {false, false}, meas.mahalanobis_threshold);
  }

  // Yaw-only pose update.
  Eigen::Matrix<double, 1, 1> z;
  z(0) = meas.theta;
  Eigen::Matrix<double, 1, 1> z_hat;
  z_hat(0) = state_.theta();
  Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, toIndex(StateIndex::kTheta)) = 1.0;
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = meas.covariance(2, 2);
  return applyUpdateFixed<1>(z, z_hat, H, R, {true}, meas.mahalanobis_threshold);
}

template <int Dim>
UpdateResult Ekf2D::updateTwistFixed(const Twist2DMeasurement& meas,
                                     const std::array<Eigen::Index, 3>& active_local,
                                     const std::array<StateIndex, 3>& active_state) noexcept {
  const Eigen::Vector3d full_z(meas.vx, meas.vy, meas.omega);
  Eigen::Matrix<double, Dim, 1> z;
  Eigen::Matrix<double, Dim, 1> z_hat;
  Eigen::Matrix<double, Dim, kStateDim> H = Eigen::Matrix<double, Dim, kStateDim>::Zero();
  Eigen::Matrix<double, Dim, Dim> R;
  std::array<bool, static_cast<std::size_t>(Dim)> angular_mask{};

  for (Eigen::Index r = 0; r < Dim; ++r) {
    const Eigen::Index local_r = active_local[static_cast<std::size_t>(r)];
    const Eigen::Index state_r = toIndex(active_state[static_cast<std::size_t>(r)]);
    z(r) = full_z(local_r);
    z_hat(r) = state_.mean(state_r);
    H(r, state_r) = 1.0;
    angular_mask[static_cast<std::size_t>(r)] = false;
    for (Eigen::Index c = 0; c < Dim; ++c) {
      const Eigen::Index local_c = active_local[static_cast<std::size_t>(c)];
      R(r, c) = meas.covariance(local_r, local_c);
    }
  }
  return applyUpdateFixed<Dim>(z, z_hat, H, R, angular_mask, meas.mahalanobis_threshold);
}

template <int Dim>
UpdateResult Ekf2D::updateImuFixed(const Imu2DMeasurement& meas,
                                   const std::array<Eigen::Index, 4>& active_rows) noexcept {
  const Eigen::Vector4d full_z(meas.yaw, meas.yaw_rate, meas.ax, meas.ay);
  const Eigen::Vector4d full_z_hat = predictImuMeasurement(state_.mean);
  const Eigen::Matrix<double, 4, kStateDim> full_H = computeImuJacobian(state_.mean);

  Eigen::Matrix<double, Dim, 1> z;
  Eigen::Matrix<double, Dim, 1> z_hat;
  Eigen::Matrix<double, Dim, kStateDim> H;
  Eigen::Matrix<double, Dim, Dim> R;
  std::array<bool, static_cast<std::size_t>(Dim)> angular_mask{};

  for (Eigen::Index r = 0; r < Dim; ++r) {
    const Eigen::Index row_r = active_rows[static_cast<std::size_t>(r)];
    z(r) = full_z(row_r);
    z_hat(r) = full_z_hat(row_r);
    H.row(r) = full_H.row(row_r);
    angular_mask[static_cast<std::size_t>(r)] = (row_r == 0);
    for (Eigen::Index c = 0; c < Dim; ++c) {
      const Eigen::Index row_c = active_rows[static_cast<std::size_t>(c)];
      R(r, c) = meas.covariance(row_r, row_c);
    }
  }
  return applyUpdateFixed<Dim>(z, z_hat, H, R, angular_mask, meas.mahalanobis_threshold);
}

UpdateResult Ekf2D::updateTwist(const Twist2DMeasurement& meas) noexcept {
  if (!state_.initialized) {
    return UpdateResult{UpdateStatus::kNotInitialized, 0.0, 0};
  }

  std::array<Eigen::Index, 3> active_local{};
  std::array<StateIndex, 3> active_state{};
  Eigen::Index count = 0;

  if (meas.use_vx) {
    active_local[static_cast<std::size_t>(count)] = 0;
    active_state[static_cast<std::size_t>(count)] = StateIndex::kVx;
    ++count;
  }
  if (meas.use_vy) {
    active_local[static_cast<std::size_t>(count)] = 1;
    active_state[static_cast<std::size_t>(count)] = StateIndex::kVy;
    ++count;
  }
  if (meas.use_omega) {
    active_local[static_cast<std::size_t>(count)] = 2;
    active_state[static_cast<std::size_t>(count)] = StateIndex::kOmega;
    ++count;
  }

  // See docs/theory/ekf_2d.md, Section 3.3.2, Eq. (9).
  switch (count) {
    case 1:
      return updateTwistFixed<1>(meas, active_local, active_state);
    case 2:
      return updateTwistFixed<2>(meas, active_local, active_state);
    case 3:
      return updateTwistFixed<3>(meas, active_local, active_state);
    default:
      return UpdateResult{UpdateStatus::kNoActiveChannels, 0.0, 0};
  }
}

UpdateResult Ekf2D::updateImu(const Imu2DMeasurement& meas) noexcept {
  if (!state_.initialized) {
    return UpdateResult{UpdateStatus::kNotInitialized, 0.0, 0};
  }

  std::array<Eigen::Index, 4> active_rows{};
  Eigen::Index count = 0;

  if (meas.use_yaw) {
    active_rows[static_cast<std::size_t>(count++)] = 0;
  }
  if (meas.use_yaw_rate) {
    active_rows[static_cast<std::size_t>(count++)] = 1;
  }
  if (meas.use_linear_accel) {
    active_rows[static_cast<std::size_t>(count++)] = 2;
    active_rows[static_cast<std::size_t>(count++)] = 3;
  }

  // See docs/theory/ekf_2d.md, Section 3.3.3, Eqs. (10)-(12).
  switch (count) {
    case 1:
      return updateImuFixed<1>(meas, active_rows);
    case 2:
      return updateImuFixed<2>(meas, active_rows);
    case 3:
      return updateImuFixed<3>(meas, active_rows);
    case 4:
      return updateImuFixed<4>(meas, active_rows);
    default:
      return UpdateResult{UpdateStatus::kNoActiveChannels, 0.0, 0};
  }
}

}  // namespace slam_sensor_fusion::core
