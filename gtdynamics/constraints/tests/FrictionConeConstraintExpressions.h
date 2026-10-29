/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file FrictionConeConstraintExpressions.h
 * @brief Expression-based reference constraints used only in tests.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/NonlinearInequalityConstraint.h>

#include <cmath>

namespace friction_test {
using namespace gtsam;

inline ScalarExpressionInequalityConstraint::shared_ptr
Pyramid(Key key, size_t edge, double sigma = 1.0) {
  const Double_ coefficient(
      [edge](const Vector4 &e, OptionalJacobian<1, 4> H) {
        if (H) {
          H->setZero();
          (*H)(0, edge) = 1;
        }
        return e(edge);
      },
      Expression<Vector4>(key));
  return ScalarExpressionInequalityConstraint::GeqZero(coefficient, sigma);
}

inline ScalarExpressionInequalityConstraint::shared_ptr
Normal(Key key, double sigma = 1.0) {
  const Double_ normal(
      [](const Vector3 &f, OptionalJacobian<1, 3> H) {
        if (H)
          *H << 0, 0, 1;
        return f(2);
      },
      Expression<Vector3>(key));
  return ScalarExpressionInequalityConstraint::GeqZero(normal, sigma);
}

inline ScalarExpressionInequalityConstraint::shared_ptr
Elliptic(Key key, const Vector2 &mu, double sigma = 1.0) {
  const Double_ residual(
      [mu](const Vector3 &f, OptionalJacobian<1, 3> H) {
        const Vector2 t = f.head<2>().cwiseQuotient(mu);
        const double norm = std::hypot(t(0), t(1));
        if (H) {
          *H << 0, 0, -1;
          if (norm > 0)
            H->leftCols<2>() = (t.cwiseQuotient(mu) / norm).transpose();
        }
        return norm - f(2);
      },
      Expression<Vector3>(key));
  return ScalarExpressionInequalityConstraint::LeqZero(residual, sigma);
}

inline ScalarExpressionInequalityConstraint::shared_ptr
SignedSquare(Key key, const Vector2 &mu, double sigma = 1.0) {
  // Use the original positive-feasible convention; GeqZero must negate it.
  const Double_ margin(
      [mu](const Vector3 &f, OptionalJacobian<1, 3> H) {
        const double sign = (f(2) > 0) - (f(2) < 0);
        const double ratio = mu(0) / mu(1);
        if (H)
          *H << -2 * f(0), -2 * ratio * ratio * f(1),
              2 * mu(0) * mu(0) * sign * f(2);
        return sign * mu(0) * mu(0) * f(2) * f(2) - f(0) * f(0) -
               ratio * ratio * f(1) * f(1);
      },
      Expression<Vector3>(key));
  return ScalarExpressionInequalityConstraint::GeqZero(margin, sigma);
}
} // namespace friction_test
