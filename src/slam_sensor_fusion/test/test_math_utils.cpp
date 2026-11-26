#include "slam_sensor_fusion/core/math_utils.hpp"

#include <Eigen/LU>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace slam_sensor_fusion::core {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;

TEST(MathUtilsTest, WrapAngle_CanonicalRange_Unchanged) {
  EXPECT_NEAR(wrapAngle(0.0), 0.0, 1.0e-15);
  EXPECT_NEAR(wrapAngle(1.2345), 1.2345, 1.0e-15);
  EXPECT_NEAR(wrapAngle(-2.5), -2.5, 1.0e-15);
  EXPECT_NEAR(wrapAngle(-kPi), -kPi, 1.0e-15);
}

TEST(MathUtilsTest, WrapAngle_BoundariesAndLargeMultiples_NormalizedToHalfOpenInterval) {
  EXPECT_NEAR(wrapAngle(kPi), -kPi, 1.0e-15);
  EXPECT_NEAR(wrapAngle(3.0 * kPi), -kPi, 1.0e-14);
  EXPECT_NEAR(wrapAngle(-3.0 * kPi), -kPi, 1.0e-14);
  EXPECT_NEAR(wrapAngle(2.0 * kPi), 0.0, 1.0e-14);
  EXPECT_NEAR(wrapAngle(-2.0 * kPi), 0.0, 1.0e-14);
  EXPECT_NEAR(wrapAngle(kPi + 0.25), -kPi + 0.25, 1.0e-14);
  EXPECT_NEAR(wrapAngle(-kPi - 0.25), kPi - 0.25, 1.0e-14);
}

TEST(MathUtilsTest, WrapAngle_NonFiniteInputs_Preserved) {
  EXPECT_TRUE(std::isnan(wrapAngle(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isinf(wrapAngle(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isinf(wrapAngle(-std::numeric_limits<double>::infinity())));
}

TEST(MathUtilsTest, IsSymmetricPositiveDefinite_ValidAndInvalidMatrices) {
  Eigen::Matrix3d spd;
  spd << 2.0, 0.3, -0.1, 0.3, 1.5, 0.2, -0.1, 0.2, 0.8;
  EXPECT_TRUE(isSymmetricPositiveDefinite(spd));

  // Zero matrix is PSD but not SPD.
  EXPECT_FALSE(isSymmetricPositiveDefinite(Eigen::Matrix3d::Zero()));

  // Indefinite matrix.
  Eigen::Matrix2d indef;
  indef << 1.0, 0.0, 0.0, -0.5;
  EXPECT_FALSE(isSymmetricPositiveDefinite(indef));

  // Asymmetric matrix.
  Eigen::Matrix2d asym;
  asym << 2.0, 0.5, 0.0, 2.0;
  EXPECT_FALSE(isSymmetricPositiveDefinite(asym));

  // Non-square and empty matrices.
  EXPECT_FALSE(isSymmetricPositiveDefinite(Eigen::Matrix<double, 2, 3>::Ones()));
  EXPECT_FALSE(isSymmetricPositiveDefinite(Eigen::MatrixXd()));

  // NaN entry.
  Eigen::Matrix2d nan_mat = Eigen::Matrix2d::Identity();
  nan_mat(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isSymmetricPositiveDefinite(nan_mat));
}

TEST(MathUtilsTest, IsSymmetricPositiveSemiDefinite_ZeroEigenvalueAcceptedNegativeRejected) {
  Eigen::Matrix2d rank1;
  rank1 << 1.0, 1.0, 1.0, 1.0;
  EXPECT_FALSE(isSymmetricPositiveDefinite(rank1));
  EXPECT_TRUE(isSymmetricPositiveSemiDefinite(rank1));

  Eigen::Matrix2d neg_eig;
  neg_eig << 1.0, 0.0, 0.0, -1.0e-3;
  EXPECT_FALSE(isSymmetricPositiveSemiDefinite(neg_eig));
}

TEST(MathUtilsTest, Rotation2D_SatisfiesSO2Invariants) {
  const double a = 0.73;
  const double b = -1.92;
  const Eigen::Matrix2d Ra = rotation2D(a);
  const Eigen::Matrix2d Rb = rotation2D(b);

  EXPECT_TRUE((Ra.transpose() * Ra).isApprox(Eigen::Matrix2d::Identity(), 1.0e-14));
  EXPECT_NEAR(Ra.determinant(), 1.0, 1.0e-14);
  EXPECT_TRUE((Ra * Rb).isApprox(rotation2D(a + b), 1.0e-14));
}

TEST(MathUtilsTest, DefaultChiSquaredThreshold99_ReturnsExpectedValues) {
  EXPECT_NEAR(defaultChiSquaredThreshold99(1), 6.635, 1.0e-12);
  EXPECT_NEAR(defaultChiSquaredThreshold99(2), 9.210, 1.0e-12);
  EXPECT_NEAR(defaultChiSquaredThreshold99(3), 11.345, 1.0e-12);
  EXPECT_NEAR(defaultChiSquaredThreshold99(4), 13.277, 1.0e-12);
  EXPECT_DOUBLE_EQ(defaultChiSquaredThreshold99(0), 0.0);
  EXPECT_DOUBLE_EQ(defaultChiSquaredThreshold99(5), 0.0);
}

}  // namespace
}  // namespace slam_sensor_fusion::core
