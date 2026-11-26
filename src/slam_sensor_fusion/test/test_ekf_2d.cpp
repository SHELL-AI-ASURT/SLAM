#include "slam_sensor_fusion/core/ekf_2d.hpp"
#include "slam_sensor_fusion/core/math_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>

namespace slam_sensor_fusion::core {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;

TEST(Ekf2DTest, Constructor_InvalidConfig_ThrowsInvalidArgument) {
  EkfConfig bad_init{};
  bad_init.initial_covariance_diag(0) = 0.0;
  EXPECT_THROW((void)Ekf2D(bad_init), std::invalid_argument);

  EkfConfig bad_q{};
  bad_q.process_noise_rate_diag(3) = -0.01;
  EXPECT_THROW((void)Ekf2D(bad_q), std::invalid_argument);

  EkfConfig bad_dt{};
  bad_dt.max_dt_sec = -1.0;
  EXPECT_THROW((void)Ekf2D(bad_dt), std::invalid_argument);
}

TEST(Ekf2DTest, Initialize_RejectsInvalidInputsAndWrapsTheta) {
  Ekf2D ekf{EkfConfig{}};
  EXPECT_FALSE(ekf.isInitialized());

  StateVector bad_mean = StateVector::Zero();
  bad_mean(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ekf.initialize(bad_mean, StateMatrix::Identity(), 0.0));
  EXPECT_FALSE(ekf.isInitialized());

  EXPECT_FALSE(ekf.initialize(StateVector::Zero(), StateMatrix::Zero(), 0.0));
  EXPECT_FALSE(ekf.initialize(StateVector::Zero(), StateMatrix::Identity(),
                              std::numeric_limits<double>::infinity()));

  StateVector valid_mean = StateVector::Zero();
  valid_mean(toIndex(StateIndex::kTheta)) = 3.0 * kPi + 0.25;
  EXPECT_TRUE(ekf.initialize(valid_mean, StateMatrix::Identity() * 0.5, 10.0));
  EXPECT_TRUE(ekf.isInitialized());
  EXPECT_NEAR(ekf.state().theta(), -kPi + 0.25, 1.0e-12);
  EXPECT_DOUBLE_EQ(ekf.state().timestamp_sec, 10.0);
}

TEST(Ekf2DTest, PropagateState_StraightConstantAcceleration_MatchesAnalyticalSolution) {
  StateVector x0 = StateVector::Zero();
  x0(toIndex(StateIndex::kX)) = 1.0;
  x0(toIndex(StateIndex::kY)) = -2.0;
  x0(toIndex(StateIndex::kTheta)) = kPi / 4.0;
  x0(toIndex(StateIndex::kVx)) = 2.0;
  x0(toIndex(StateIndex::kAx)) = 1.5;

  const double dt = 0.4;
  const StateVector x1 = Ekf2D::propagateState(x0, dt);

  const double expected_dist = 2.0 * dt + 0.5 * 1.5 * dt * dt;
  const double c = std::cos(kPi / 4.0);
  const double s = std::sin(kPi / 4.0);

  EXPECT_NEAR(x1(toIndex(StateIndex::kX)), 1.0 + expected_dist * c, 1.0e-14);
  EXPECT_NEAR(x1(toIndex(StateIndex::kY)), -2.0 + expected_dist * s, 1.0e-14);
  EXPECT_NEAR(x1(toIndex(StateIndex::kTheta)), kPi / 4.0, 1.0e-14);
  EXPECT_NEAR(x1(toIndex(StateIndex::kVx)), 2.0 + 1.5 * dt, 1.0e-14);
}

TEST(Ekf2DTest, Predict_CircularTurn_TracksExactArcWithMidpointIntegration) {
  EkfConfig cfg{};
  cfg.max_substep_dt_sec = 0.01;
  Ekf2D ekf(cfg);

  const double v = 3.0;
  const double omega = 0.5;
  const double radius = v / omega;  // 6.0 m
  const double duration = kPi / (2.0 * omega);  // 90 deg turn

  StateVector x0 = StateVector::Zero();
  x0(toIndex(StateIndex::kVx)) = v;
  x0(toIndex(StateIndex::kOmega)) = omega;
  ASSERT_TRUE(ekf.initialize(x0, StateMatrix::Identity() * 0.01, 0.0));

  const int steps = 50;
  const double dt = duration / static_cast<double>(steps);
  for (int i = 0; i < steps; ++i) {
    ASSERT_EQ(ekf.predict(dt), PredictStatus::kPredicted);
  }

  // Starting at (0, 0, 0) turning CCW with radius R: after 90 deg, pose is (R, R, pi/2).
  // Midpoint discretization error over 10 ms substeps is O(omega^2 * dt^2) < 1e-4 m.
  EXPECT_NEAR(ekf.state().x(), radius, 1.0e-3);
  EXPECT_NEAR(ekf.state().y(), radius, 1.0e-3);
  EXPECT_NEAR(ekf.state().theta(), kPi / 2.0, 1.0e-12);
  EXPECT_TRUE(isSymmetricPositiveDefinite(ekf.state().covariance));
}

TEST(Ekf2DTest, ComputeProcessJacobian_MatchesCentralFiniteDifferences) {
  StateVector x;
  x << 4.2, -3.1, 0.85, 3.2, -0.4, 0.65, 1.1, -0.35;
  const double dt = 0.05;

  const StateMatrix F_analytic = Ekf2D::computeProcessJacobian(x, dt);

  // Central finite differences with eps = 1e-6 achieve O(eps^2) ~= 1e-12 truncation error;
  // tolerance 1e-7 leaves comfortable floating-point safety margin while catching any sign/term bug.
  constexpr double kEps = 1.0e-6;
  StateMatrix F_numeric = StateMatrix::Zero();
  for (Eigen::Index j = 0; j < kStateDim; ++j) {
    StateVector x_plus = x;
    StateVector x_minus = x;
    x_plus(j) += kEps;
    x_minus(j) -= kEps;

    const StateVector f_plus = Ekf2D::propagateState(x_plus, dt);
    const StateVector f_minus = Ekf2D::propagateState(x_minus, dt);
    StateVector diff = f_plus - f_minus;
    diff(toIndex(StateIndex::kTheta)) = wrapAngle(diff(toIndex(StateIndex::kTheta)));
    F_numeric.col(j) = diff / (2.0 * kEps);
  }

  for (Eigen::Index r = 0; r < kStateDim; ++r) {
    for (Eigen::Index c = 0; c < kStateDim; ++c) {
      EXPECT_NEAR(F_analytic(r, c), F_numeric(r, c), 1.0e-7)
          << "Mismatch at F(" << r << ", " << c << ")";
    }
  }
}

TEST(Ekf2DTest, ComputeImuJacobian_MatchesCentralFiniteDifferences) {
  StateVector x;
  x << -1.5, 2.7, -1.1, 3.8, -0.25, -0.45, 0.9, -0.6;

  const Eigen::Matrix<double, 4, kStateDim> H_analytic = Ekf2D::computeImuJacobian(x);

  constexpr double kEps = 1.0e-6;
  Eigen::Matrix<double, 4, kStateDim> H_numeric = Eigen::Matrix<double, 4, kStateDim>::Zero();
  for (Eigen::Index j = 0; j < kStateDim; ++j) {
    StateVector x_plus = x;
    StateVector x_minus = x;
    x_plus(j) += kEps;
    x_minus(j) -= kEps;
    H_numeric.col(j) =
        (Ekf2D::predictImuMeasurement(x_plus) - Ekf2D::predictImuMeasurement(x_minus)) /
        (2.0 * kEps);
  }

  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < kStateDim; ++c) {
      EXPECT_NEAR(H_analytic(r, c), H_numeric(r, c), 1.0e-8)
          << "Mismatch at H_imu(" << r << ", " << c << ")";
    }
  }
}

TEST(Ekf2DTest, UpdateImu_SteadyTurnCentripetalAccel_PreservesZeroLateralVelocity) {
  Ekf2D ekf{EkfConfig{}};
  StateVector x0 = StateVector::Zero();
  x0(toIndex(StateIndex::kVx)) = 4.0;
  x0(toIndex(StateIndex::kVy)) = 0.0;
  x0(toIndex(StateIndex::kOmega)) = 0.5;
  ASSERT_TRUE(ekf.initialize(x0, StateMatrix::Identity() * 0.1, 0.0));

  // True horizontal specific force in a steady circular turn: ax = 0, ay = +omega * vx = +2.0 m/s^2.
  Imu2DMeasurement imu{};
  imu.use_yaw = false;
  imu.use_yaw_rate = true;
  imu.use_linear_accel = true;
  imu.yaw_rate = 0.5;
  imu.ax = 0.0;
  imu.ay = 2.0;
  imu.covariance = Eigen::Matrix4d::Identity() * 0.01;

  const UpdateResult res = ekf.updateImu(imu);
  ASSERT_EQ(res.status, UpdateStatus::kUpdated);
  EXPECT_NEAR(res.mahalanobis_sq, 0.0, 1.0e-12);
  EXPECT_NEAR(ekf.state().vy(), 0.0, 1.0e-12);
  EXPECT_NEAR(ekf.state().ay(), 0.0, 1.0e-12);
}

TEST(Ekf2DTest, UpdatePose_WraparoundAcrossPi_UpdatesAlongShortestArc) {
  Ekf2D ekf{EkfConfig{}};
  StateVector x0 = StateVector::Zero();
  x0(toIndex(StateIndex::kTheta)) = kPi - 0.05;  // +3.0916 rad
  ASSERT_TRUE(ekf.initialize(x0, StateMatrix::Identity() * 0.1, 0.0));

  Pose2DMeasurement pose{};
  pose.x = 0.0;
  pose.y = 0.0;
  pose.theta = -kPi + 0.05;  // -3.0916 rad (0.10 rad CCW past x0.theta)
  pose.covariance = Eigen::Matrix3d::Identity() * 0.1;  // Equal weight -> midpoint at -pi.

  const UpdateResult res = ekf.updatePose(pose);
  ASSERT_EQ(res.status, UpdateStatus::kUpdated);
  // Shortest-arc midpoint between (+pi - 0.05) and (-pi + 0.05) is -pi (NOT 0.0).
  EXPECT_NEAR(std::abs(ekf.state().theta()), kPi, 1.0e-6);
}

TEST(Ekf2DTest, UpdatePose_JosephFormStability_PreservesPsdUnderExtremeConditioning) {
  Ekf2D ekf{EkfConfig{}};
  ASSERT_TRUE(ekf.initialize(StateVector::Zero(), StateMatrix::Identity() * 1.0e6, 0.0));

  Pose2DMeasurement pose{};
  pose.x = 1.0;
  pose.y = 2.0;
  pose.theta = 0.5;
  pose.covariance = Eigen::Matrix3d::Identity() * 1.0e-10;
  pose.mahalanobis_threshold = 0.0;  // Disable gating to force extreme Kalman gain.

  const UpdateResult res = ekf.updatePose(pose);
  ASSERT_EQ(res.status, UpdateStatus::kUpdated);
  EXPECT_NEAR(ekf.state().x(), 1.0, 1.0e-6);
  EXPECT_NEAR(ekf.state().y(), 2.0, 1.0e-6);
  EXPECT_NEAR(ekf.state().theta(), 0.5, 1.0e-6);
  EXPECT_TRUE(isSymmetricPositiveSemiDefinite(ekf.state().covariance, 1.0e-9, -1.0e-12));
}

TEST(Ekf2DTest, UpdatePose_OutlierSpike_RejectedWithoutMutatingState) {
  Ekf2D ekf{EkfConfig{}};
  ASSERT_TRUE(ekf.initialize(StateVector::Zero(), StateMatrix::Identity() * 0.01, 0.0));

  const StateVector mean_before = ekf.state().mean;
  const StateMatrix cov_before = ekf.state().covariance;

  Pose2DMeasurement outlier{};
  outlier.x = 15.0;  // ~150 sigma jump
  outlier.y = -10.0;
  outlier.theta = 0.0;
  outlier.covariance = Eigen::Matrix3d::Identity() * 0.01;
  outlier.mahalanobis_threshold = defaultChiSquaredThreshold99(3);

  const UpdateResult res = ekf.updatePose(outlier);
  EXPECT_EQ(res.status, UpdateStatus::kRejectedOutlier);
  EXPECT_GT(res.mahalanobis_sq, outlier.mahalanobis_threshold);
  EXPECT_TRUE(ekf.state().mean.isApprox(mean_before, 0.0));
  EXPECT_TRUE(ekf.state().covariance.isApprox(cov_before, 0.0));
}

TEST(Ekf2DTest, EdgeCases_InvalidMeasurementsAndTimingRejectedCleanly) {
  Ekf2D ekf{EkfConfig{}};
  EXPECT_EQ(ekf.predict(0.01), PredictStatus::kNotInitialized);
  EXPECT_EQ(ekf.updatePose(Pose2DMeasurement{}).status, UpdateStatus::kNotInitialized);

  ASSERT_TRUE(ekf.initializeAtOrigin(1.0));

  EXPECT_EQ(ekf.predict(-0.01), PredictStatus::kInvalidDt);
  EXPECT_EQ(ekf.predict(10.0), PredictStatus::kInvalidDt);
  EXPECT_EQ(ekf.predict(std::numeric_limits<double>::quiet_NaN()), PredictStatus::kInvalidDt);
  EXPECT_EQ(ekf.predict(0.0), PredictStatus::kSkippedZeroDt);
  EXPECT_EQ(ekf.predictTo(0.5), PredictStatus::kInvalidDt);

  Pose2DMeasurement disabled_pose{};
  disabled_pose.use_position = false;
  disabled_pose.use_yaw = false;
  EXPECT_EQ(ekf.updatePose(disabled_pose).status, UpdateStatus::kNoActiveChannels);

  Twist2DMeasurement nan_twist{};
  nan_twist.vx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ekf.updateTwist(nan_twist).status, UpdateStatus::kInvalidMeasurement);

  Twist2DMeasurement zero_cov_twist{};
  zero_cov_twist.covariance = Eigen::Matrix3d::Zero();
  EXPECT_EQ(ekf.updateTwist(zero_cov_twist).status, UpdateStatus::kInvalidCovariance);

  Imu2DMeasurement disabled_imu{};
  disabled_imu.use_yaw = false;
  disabled_imu.use_yaw_rate = false;
  disabled_imu.use_linear_accel = false;
  EXPECT_EQ(ekf.updateImu(disabled_imu).status, UpdateStatus::kNoActiveChannels);
}

TEST(Ekf2DTest, SyntheticMultiSensorFusion_ImprovesAccuracyRejectsOutliersAndSurvivesDropout) {
  // Deterministic PRNG seed per rules.md Section 1.4 & 4.2.
  std::mt19937 rng(42U);

  const double sigma_lidar_xy = 0.15;
  const double sigma_lidar_yaw = 0.04;
  const double sigma_wheel_vx = 0.10;
  const double sigma_wheel_vy = 0.03;
  const double sigma_wheel_w = 0.05;
  const double sigma_imu_w = 0.015;
  const double sigma_imu_acc = 0.20;

  std::normal_distribution<double> n_lidar_xy(0.0, sigma_lidar_xy);
  std::normal_distribution<double> n_lidar_yaw(0.0, sigma_lidar_yaw);
  std::normal_distribution<double> n_wheel_vx(0.0, sigma_wheel_vx);
  std::normal_distribution<double> n_wheel_vy(0.0, sigma_wheel_vy);
  std::normal_distribution<double> n_wheel_w(0.0, sigma_wheel_w);
  std::normal_distribution<double> n_imu_w(0.0, sigma_imu_w);
  std::normal_distribution<double> n_imu_acc(0.0, sigma_imu_acc);

  EkfConfig cfg{};
  cfg.process_noise_rate_diag << 0.005, 0.005, 0.002, 0.20, 0.02, 0.05, 1.5, 0.5;
  Ekf2D ekf(cfg);

  // Ground truth: vehicle accelerates smoothly then drives a curve (realistic Shell car speed).
  StateVector truth = StateVector::Zero();
  truth(toIndex(StateIndex::kVx)) = 2.0;
  ASSERT_TRUE(ekf.initialize(truth, StateMatrix::Identity() * 0.05, 0.0));

  constexpr double kDt = 0.01;  // 100 Hz simulation
  constexpr int kTotalSteps = 1000;  // 10.0 s

  double fused_pos_sq_err = 0.0;
  double raw_lidar_pos_sq_err = 0.0;
  int eval_count = 0;
  int raw_lidar_count = 0;
  int outlier_rejections = 0;

  for (int step = 1; step <= kTotalSteps; ++step) {
    const double t = static_cast<double>(step) * kDt;

    // Smoothly varying true acceleration & turn rate.
    const double true_ax = (t < 3.0) ? 0.5 : ((t > 7.0) ? -0.3 : 0.0);
    const double true_omega = 0.35 * std::sin(0.6 * t);
    truth(toIndex(StateIndex::kAx)) = true_ax;
    truth(toIndex(StateIndex::kAy)) = 0.0;
    truth(toIndex(StateIndex::kOmega)) = true_omega;
    truth = Ekf2D::propagateState(truth, kDt);

    ASSERT_EQ(ekf.predict(kDt), PredictStatus::kPredicted);

    // 100 Hz IMU update (yaw rate + Coriolis horizontal specific force).
    Imu2DMeasurement imu{};
    imu.use_yaw = false;
    imu.use_yaw_rate = true;
    imu.use_linear_accel = true;
    const Eigen::Vector4d true_imu = Ekf2D::predictImuMeasurement(truth);
    imu.yaw_rate = true_imu(1) + n_imu_w(rng);
    imu.ax = true_imu(2) + n_imu_acc(rng);
    imu.ay = true_imu(3) + n_imu_acc(rng);
    imu.covariance =
        Eigen::Vector4d(0.1, sigma_imu_w * sigma_imu_w, sigma_imu_acc * sigma_imu_acc,
                        sigma_imu_acc * sigma_imu_acc)
            .asDiagonal();
    (void)ekf.updateImu(imu);

    // 33 Hz wheel odometry twist update.
    if (step % 3 == 0) {
      Twist2DMeasurement wheel{};
      wheel.vx = truth(toIndex(StateIndex::kVx)) + n_wheel_vx(rng);
      wheel.vy = 0.0 + n_wheel_vy(rng);
      wheel.omega = truth(toIndex(StateIndex::kOmega)) + n_wheel_w(rng);
      wheel.covariance = Eigen::Vector3d(sigma_wheel_vx * sigma_wheel_vx,
                                         sigma_wheel_vy * sigma_wheel_vy,
                                         sigma_wheel_w * sigma_wheel_w)
                             .asDiagonal();
      (void)ekf.updateTwist(wheel);
    }

    // 10 Hz LiDAR pose update, with a simulated dropout during t in [4.0, 7.0] and 2 outlier spikes.
    const bool lidar_dropout = (t >= 4.0 && t <= 7.0);
    if (step % 10 == 0 && !lidar_dropout) {
      Pose2DMeasurement lidar{};
      const bool inject_outlier = (step == 200 || step == 850);
      lidar.x = truth(toIndex(StateIndex::kX)) + n_lidar_xy(rng) + (inject_outlier ? 8.0 : 0.0);
      lidar.y = truth(toIndex(StateIndex::kY)) + n_lidar_xy(rng) + (inject_outlier ? -6.0 : 0.0);
      lidar.theta = wrapAngle(truth(toIndex(StateIndex::kTheta)) + n_lidar_yaw(rng));
      lidar.covariance = Eigen::Vector3d(sigma_lidar_xy * sigma_lidar_xy,
                                         sigma_lidar_xy * sigma_lidar_xy,
                                         sigma_lidar_yaw * sigma_lidar_yaw)
                             .asDiagonal();
      lidar.mahalanobis_threshold = defaultChiSquaredThreshold99(3);

      const UpdateResult res = ekf.updatePose(lidar);
      if (inject_outlier) {
        EXPECT_EQ(res.status, UpdateStatus::kRejectedOutlier);
        if (res.status == UpdateStatus::kRejectedOutlier) {
          ++outlier_rejections;
        }
      } else {
        const double rx = lidar.x - truth(toIndex(StateIndex::kX));
        const double ry = lidar.y - truth(toIndex(StateIndex::kY));
        raw_lidar_pos_sq_err += rx * rx + ry * ry;
        ++raw_lidar_count;
      }
    }

    // Verify covariance invariant periodically and accumulate position error.
    if (step % 25 == 0) {
      ASSERT_TRUE(isSymmetricPositiveSemiDefinite(ekf.state().covariance));
    }
    const double ex = ekf.state().x() - truth(toIndex(StateIndex::kX));
    const double ey = ekf.state().y() - truth(toIndex(StateIndex::kY));
    fused_pos_sq_err += ex * ex + ey * ey;
    ++eval_count;
  }

  const double fused_rmse = std::sqrt(fused_pos_sq_err / static_cast<double>(eval_count));
  const double raw_lidar_rmse =
      std::sqrt(raw_lidar_pos_sq_err / static_cast<double>(raw_lidar_count));

  EXPECT_EQ(outlier_rejections, 2);
  // Even with a 3-second LiDAR dropout and injected outliers, fused RMSE stays well below raw noise.
  EXPECT_LT(fused_rmse, raw_lidar_rmse);
  EXPECT_LT(fused_rmse, 0.15);
}

}  // namespace
}  // namespace slam_sensor_fusion::core
