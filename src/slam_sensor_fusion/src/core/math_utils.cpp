#include "slam_sensor_fusion/core/math_utils.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <numbers>

namespace slam_sensor_fusion::core {

namespace {
constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

double wrapAngle(const double angle_rad) noexcept {
  if (!std::isfinite(angle_rad)) {
    return angle_rad;
  }
  // std::remainder maps into [-pi, pi]. Shift +pi strictly to -pi to enforce [-pi, pi).
  double wrapped = std::remainder(angle_rad, kTwoPi);
  if (wrapped >= kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped < -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

bool isFiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat) noexcept {
  return mat.allFinite();
}

bool isSymmetricPositiveDefinite(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                                 const double symmetry_tol,
                                 const double min_pivot) noexcept {
  if (mat.rows() == 0 || mat.rows() != mat.cols()) {
    return false;
  }
  if (!std::isfinite(symmetry_tol) || symmetry_tol < 0.0 || !std::isfinite(min_pivot) ||
      min_pivot <= 0.0) {
    return false;
  }
  if (!mat.allFinite()) {
    return false;
  }
  const double max_asymmetry = (mat - mat.transpose()).cwiseAbs().maxCoeff();
  if (max_asymmetry > symmetry_tol) {
    return false;
  }

  const Eigen::MatrixXd sym = 0.5 * (mat + mat.transpose());
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(sym);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return false;
  }
  return (ldlt.vectorD().array() > min_pivot).all();
}

bool isSymmetricPositiveSemiDefinite(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                                     const double symmetry_tol,
                                     const double min_eigenvalue) noexcept {
  if (mat.rows() == 0 || mat.rows() != mat.cols()) {
    return false;
  }
  if (!std::isfinite(symmetry_tol) || symmetry_tol < 0.0 || !std::isfinite(min_eigenvalue)) {
    return false;
  }
  if (!mat.allFinite()) {
    return false;
  }
  const double max_asymmetry = (mat - mat.transpose()).cwiseAbs().maxCoeff();
  if (max_asymmetry > symmetry_tol) {
    return false;
  }

  const Eigen::MatrixXd sym = 0.5 * (mat + mat.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(sym, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  return (solver.eigenvalues().array() >= min_eigenvalue).all();
}

Eigen::Matrix2d rotation2D(const double angle_rad) noexcept {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix2d rot;
  rot(0, 0) = c;
  rot(0, 1) = -s;
  rot(1, 0) = s;
  rot(1, 1) = c;
  return rot;
}

double defaultChiSquaredThreshold99(const Eigen::Index dof) noexcept {
  switch (dof) {
    case 1:
      return 6.635;
    case 2:
      return 9.210;
    case 3:
      return 11.345;
    case 4:
      return 13.277;
    default:
      return 0.0;
  }
}

}  // namespace slam_sensor_fusion::core
