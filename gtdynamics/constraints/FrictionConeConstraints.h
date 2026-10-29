/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file FrictionConeConstraints.h
 * @brief Scalar GTSAM inequalities for pyramidal and elliptic friction cones.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/NonlinearInequalityConstraint.h>

namespace gtdynamics {

/** One edge nonnegativity constraint, -edges[edge] <= 0, of a friction pyramid.
 *
 * Create() supplies all four scalar constraints, so CMC can activate edges
 * independently. The Vector4 coefficients describe forces along the four rays
 * (+mu_x,0,1), (-mu_x,0,1), (0,+mu_y,1), (0,-mu_y,1). ForceMatrix() maps them
 * to (t_x,t_y,n). This gives |t_x|/mu_x + |t_y|/mu_y <= n, not an elliptic
 * cone. The key stores a Vector4 of edge coefficients.
 */
class PyramidalFrictionConeConstraint
    : public gtsam::NonlinearInequalityConstraint {
  size_t edge_;

public:
  PyramidalFrictionConeConstraint(gtsam::Key key, size_t edge,
                                  double sigma = 1.0);

  /// Construct the complete pyramid as four independently active inequalities.
  static gtsam::NonlinearInequalityConstraints Create(gtsam::Key key,
                                                      double sigma = 1.0);

  /// Map four edge coefficients to (t_x,t_y,n), for positive finite mu.
  static gtsam::Matrix34 ForceMatrix(const gtsam::Vector2 &mu);

  /// Evaluate the signed residual and its analytic Jacobian directly.
  gtsam::Vector
  unwhitenedExpr(const gtsam::Values &values,
                 gtsam::OptionalMatrixVecType H = nullptr) const override;

  /// Preserve the signed residual, Jacobian, and sigma on a blocking face.
  gtsam::NonlinearEqualityConstraint::shared_ptr
  createEqualityConstraint() const override;

  /// Apply a custom scalar penalty without constructing an expression tree.
  gtsam::NoiseModelFactor::shared_ptr
  penaltyFactorCustom(gtsam::InequalityPenaltyFunction::shared_ptr function,
                      double mu = 1.0) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

/** Elliptic Coulomb cone: hypot(t_x/mu_x,t_y/mu_y) - n <= 0.
 *
 * The force is a Vector3 ordered (t_x,t_y,n) in the contact frame, with
 * positive n pushing against the surface. This inequality already implies n >=
 * 0. Create() also adds the redundant -n <= 0 row used in the contact
 * comparison. mu must be positive and finite; sigma scales the force-valued
 * residual.
 *
 * At zero tangential force the norm is nondifferentiable. We choose zero
 * tangential derivatives and normal derivative -1. This is a subgradient,
 * not a full description of the tangent cone at the zero-force apex.
 */
class EllipticFrictionConeConstraint
    : public gtsam::NonlinearInequalityConstraint {
  gtsam::Vector2 inverseMu_;

public:
  EllipticFrictionConeConstraint(gtsam::Key key, const gtsam::Vector2 &mu,
                                 double sigma = 1.0);

  /// Normal nonnegativity followed by the elliptic inequality, both scalar.
  static gtsam::NonlinearInequalityConstraints
  Create(gtsam::Key key, const gtsam::Vector2 &mu, double sigma = 1.0);

  /// Evaluate the signed residual and its analytic Jacobian directly.
  gtsam::Vector
  unwhitenedExpr(const gtsam::Values &values,
                 gtsam::OptionalMatrixVecType H = nullptr) const override;

  /// Preserve the signed residual, Jacobian, and sigma on a blocking face.
  gtsam::NonlinearEqualityConstraint::shared_ptr
  createEqualityConstraint() const override;

  /// Apply a custom scalar penalty without constructing an expression tree.
  gtsam::NoiseModelFactor::shared_ptr
  penaltyFactorCustom(gtsam::InequalityPenaltyFunction::shared_ptr function,
                      double mu = 1.0) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

/** Signed-square representation of the same elliptic Coulomb cone.
 *
 * For a contact-frame Vector3 (t_x,t_y,n), the inequality is
 * t_x^2 + (mu_x/mu_y)^2 t_y^2 - mu_x^2 n|n| <= 0. With isotropic friction it
 * matches the existing quadruped CMC constraint, including its residual scale.
 * The signed square excludes negative normal forces without an extra row.
 * mu and sigma must be positive and finite; this residual has force-squared
 * units, so sigma is not interchangeable with the norm formulation's sigma.
 *
 * The expression is differentiable everywhere, but its gradient vanishes at
 * the apex. Its linearization there imposes no restriction on the step.
 * The key stores a Vector3 contact force; this constraint makes no
 * affine-equality or retractor assumptions.
 */
class SignedSquareFrictionConeConstraint
    : public gtsam::NonlinearInequalityConstraint {
  double ratioSquared_, muSquared_;

public:
  SignedSquareFrictionConeConstraint(gtsam::Key key, const gtsam::Vector2 &mu,
                                     double sigma = 1.0);

  /// Evaluate the signed residual and its analytic Jacobian directly.
  gtsam::Vector
  unwhitenedExpr(const gtsam::Values &values,
                 gtsam::OptionalMatrixVecType H = nullptr) const override;

  /// Preserve the signed residual, Jacobian, and sigma on a blocking face.
  gtsam::NonlinearEqualityConstraint::shared_ptr
  createEqualityConstraint() const override;

  /// Apply a custom scalar penalty without constructing an expression tree.
  gtsam::NoiseModelFactor::shared_ptr
  penaltyFactorCustom(gtsam::InequalityPenaltyFunction::shared_ptr function,
                      double mu = 1.0) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

} // namespace gtdynamics
