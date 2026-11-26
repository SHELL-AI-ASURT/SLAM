#include "slam_sensor_fusion/core/differential_adapter.hpp"
#include "slam_sensor_fusion/core/math_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace slam_sensor_fusion::core {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;

TEST(DifferentialAdapterTest, Constructor_InvalidConfig_Throws) {
  DifferentialAdapterConfig bad_dt{};
  bad_dt.min_dt_sec = 0.0;
  EXPECT_THROW((void)DifferentialAdapter(bad_dt), std::invalid_argument);

  DifferentialAdapterConfig bad_cov{};
  bad_cov.default_step_covariance = Eigen::Matrix3d::Zero();
  EXPECT_THROW((void)DifferentialAdapter(bad_cov), std::invalid_argument);
}

TEST(DifferentialAdapterTest, Process_StraightAndTurnAcrossPi_RecoversExactBodyTwist) {
  DifferentialAdapterConfig cfg{};
  cfg.default_step_covariance = Eigen::Vector3d(0.01, 0.04, 0.002).asDiagonal();
  DifferentialAdapter adapter(cfg);
  EXPECT_FALSE(adapter.hasPrevious());

  // Anchor heading near +pi, moving forward along heading while turning CCW across +/-pi.
  const double theta0 = kPi - 0.1;
  const double dtheta = 0.2;  // Crosses +pi to (-pi + 0.1)
  const double theta1 = wrapAngle(theta0 + dtheta);
  const double theta_mid = wrapAngle(theta0 + 0.5 * dtheta);  // Exactly -pi
  const double dt = 0.1;
  const double vx_true = 3.0;
  const double vy_true = 0.2;

  const Eigen::Vector2d dp_world = rotation2D(theta_mid) * Eigen::Vector2d(vx_true * dt, vy_true * dt);

  Pose2DMeasurement p0{};
  p0.x = 5.0;
  p0.y = -2.0;
  p0.theta = theta0;
  const auto out0 = adapter.process(p0, 1.0);
  EXPECT_EQ(out0.status, DifferentialStatus::kInitializedFirstSample);
  EXPECT_FALSE(out0.hasTwist());
  EXPECT_TRUE(adapter.hasPrevious());

  Pose2DMeasurement p1{};
  p1.x = p0.x + dp_world.x();
  p1.y = p0.y + dp_world.y();
  p1.theta = theta1;
  const auto out1 = adapter.process(p1, 1.0 + dt);
  ASSERT_EQ(out1.status, DifferentialStatus::kReady);
  ASSERT_TRUE(out1.hasTwist());

  EXPECT_NEAR(out1.twist.vx, vx_true, 1.0e-12);
  EXPECT_NEAR(out1.twist.vy, vy_true, 1.0e-12);
  EXPECT_NEAR(out1.twist.omega, dtheta / dt, 1.0e-12);
  EXPECT_TRUE(isSymmetricPositiveDefinite(out1.twist.covariance));
  // Covariance scaled by 1/dt^2 = 100.
  EXPECT_NEAR(out1.twist.covariance(2, 2), 0.002 / (dt * dt), 1.0e-12);
}

TEST(DifferentialAdapterTest, Process_StaleTinyAndDropoutGaps_HandledSafely) {
  DifferentialAdapterConfig cfg{};
  cfg.min_dt_sec = 0.01;
  cfg.max_dt_sec = 0.5;
  DifferentialAdapter adapter(cfg);

  Pose2DMeasurement p0{};
  p0.x = 0.0;
  p0.y = 0.0;
  p0.theta = 0.0;
  ASSERT_EQ(adapter.process(p0, 1.0).status, DifferentialStatus::kInitializedFirstSample);

  // Stale timestamp (dt <= 0) must not alter anchor.
  Pose2DMeasurement p_stale = p0;
  p_stale.x = 100.0;
  EXPECT_EQ(adapter.process(p_stale, 0.9).status, DifferentialStatus::kStaleTimestamp);
  EXPECT_EQ(adapter.process(p_stale, 1.0).status, DifferentialStatus::kStaleTimestamp);

  // Tiny dt (< min_dt_sec) must not advance anchor.
  Pose2DMeasurement p_tiny = p0;
  p_tiny.x = 0.001;
  EXPECT_EQ(adapter.process(p_tiny, 1.001).status, DifferentialStatus::kInvalidDt);

  // Non-finite input must not corrupt anchor.
  Pose2DMeasurement p_nan = p0;
  p_nan.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(adapter.process(p_nan, 1.1).status, DifferentialStatus::kInvalidInput);

  // Valid sample at t = 1.1 still differentiates cleanly from original anchor at (0, 0, 0, t=1.0).
  Pose2DMeasurement p1 = p0;
  p1.x = 0.2;
  const auto out1 = adapter.process(p1, 1.1);
  ASSERT_EQ(out1.status, DifferentialStatus::kReady);
  EXPECT_NEAR(out1.twist.vx, 2.0, 1.0e-12);

  // Large dropout gap (> max_dt_sec) re-anchors so the subsequent sample succeeds immediately.
  Pose2DMeasurement p_after_gap = p0;
  p_after_gap.x = 10.0;
  EXPECT_EQ(adapter.process(p_after_gap, 5.0).status, DifferentialStatus::kInvalidDt);

  Pose2DMeasurement p_resumed = p0;
  p_resumed.x = 10.3;
  const auto out_resumed = adapter.process(p_resumed, 5.1);
  ASSERT_EQ(out_resumed.status, DifferentialStatus::kReady);
  EXPECT_NEAR(out_resumed.twist.vx, 3.0, 1.0e-12);

  adapter.reset();
  EXPECT_FALSE(adapter.hasPrevious());
}

}  // namespace
}  // namespace slam_sensor_fusion::core
