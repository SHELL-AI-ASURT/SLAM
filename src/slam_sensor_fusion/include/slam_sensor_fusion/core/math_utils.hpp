#pragma once

#include <Eigen/Core>

namespace slam_sensor_fusion::core {

/// @brief Wrap a planar angle into the half-open canonical interval [-pi, pi).
///
/// Preserves non-finite inputs (NaN/Inf) unchanged so upstream validation catches them explicitly.
/// @param angle_rad Input angle in radians.
/// @return Equivalent angle in [-pi, pi) [rad], or unchanged input if non-finite.
/// @see docs/theory/ekf_2d.md, Section 3.1, Eq. (1).
[[nodiscard]] double wrapAngle(double angle_rad) noexcept;

/// @brief Check whether every entry in a matrix or vector is finite (neither NaN nor Inf).
/// @param mat Matrix or vector to validate.
/// @return True iff all elements satisfy std::isfinite.
[[nodiscard]] bool isFiniteMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat) noexcept;

/// @brief Check whether a square matrix is finite, symmetric, and strictly positive definite.
///
/// Symmetry is verified via max-norm |M - M^T|_inf <= symmetry_tol, and positive definiteness
/// is checked via Eigen::LDLT factorization on the symmetrized matrix with all pivots > min_pivot.
/// @param mat Square matrix to validate.
/// @param symmetry_tol Maximum allowed element-wise asymmetry |M_ij - M_ji|.
/// @param min_pivot Minimum strictly positive LDLT pivot threshold.
/// @return True iff mat is non-empty, square, finite, symmetric, and strictly positive definite.
[[nodiscard]] bool isSymmetricPositiveDefinite(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                                               double symmetry_tol = 1.0e-6,
                                               double min_pivot = 1.0e-12) noexcept;

/// @brief Check whether a square matrix is finite, symmetric, and positive semi-definite.
///
/// Uses Eigen::SelfAdjointEigenSolver on the symmetrized matrix to verify all eigenvalues
/// satisfy lambda_min >= min_eigenvalue.
/// @param mat Square matrix to validate.
/// @param symmetry_tol Maximum allowed element-wise asymmetry |M_ij - M_ji|.
/// @param min_eigenvalue Small negative numerical tolerance for zero eigenvalues (e.g., -1e-10).
/// @return True iff mat is non-empty, square, finite, symmetric, and positive semi-definite.
[[nodiscard]] bool isSymmetricPositiveSemiDefinite(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                                                   double symmetry_tol = 1.0e-6,
                                                   double min_eigenvalue = -1.0e-10) noexcept;

/// @brief Construct the 2x2 planar rotation matrix R(angle_rad) in SO(2).
/// @param angle_rad Rotation angle (CCW positive) [rad].
/// @return 2x2 orthogonal rotation matrix with det(R) = +1.
[[nodiscard]] Eigen::Matrix2d rotation2D(double angle_rad) noexcept;

/// @brief Return the standard 99% chi-squared gating threshold for 1 to 4 degrees of freedom.
/// @param dof Measurement dimension m in [1, 4].
/// @return 99% chi-squared quantile (6.635, 9.210, 11.345, 13.277), or 0.0 if out of range.
/// @see docs/theory/ekf_2d.md, Section 3.4, Eq. (16).
[[nodiscard]] double defaultChiSquaredThreshold99(Eigen::Index dof) noexcept;

}  // namespace slam_sensor_fusion::core
