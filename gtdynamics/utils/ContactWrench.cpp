/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file ContactWrench.cpp
 * @brief Construct rectangular sole-wrench cone matrices and constraints.
 * @author GTDynamics contributors
 */

#include <gtdynamics/utils/ContactWrench.h>
#include <gtsam/constrained/LinearConstraint.h>

#include <cmath>
#include <stdexcept>

namespace gtdynamics {

RectangularContactWrench::RectangularContactWrench(double halfLength,
                                                   double halfWidth,
                                                   double frictionCoefficient)
    : halfLength_(halfLength),
      halfWidth_(halfWidth),
      frictionCoefficient_(frictionCoefficient) {
  if (!std::isfinite(halfLength) || halfLength <= 0 ||
      !std::isfinite(halfWidth) || halfWidth <= 0 ||
      !std::isfinite(frictionCoefficient) || frictionCoefficient <= 0)
    throw std::invalid_argument(
        "Support dimensions and friction must be positive and finite");
}

double RectangularContactWrench::pyramidCoefficient() const {
  return frictionCoefficient_ / std::sqrt(2.0);
}

gtsam::Matrix RectangularContactWrench::matrix() const {
  const double x = halfLength_, y = halfWidth_, mu = pyramidCoefficient();
  gtsam::Matrix result = gtsam::Matrix::Zero(17, 6);
  result(0, 5) = -1;
  size_t row = 1;
  for (const size_t axis : {3, 4}) {
    for (const double sign : {-1., 1.}) {
      result(row, axis) = sign;
      result(row++, 5) = -mu;
    }
  }
  for (const double sign : {-1., 1.}) {
    result(row, 0) = sign;
    result(row++, 5) = -y;
  }
  for (const double sign : {-1., 1.}) {
    result(row, 1) = sign;
    result(row++, 5) = -x;
  }

  // Expanding the two absolute values in each coupled yaw bound produces
  // four independent affine faces per sign. This retains a linear feasible
  // set for ReducedALRetractor rather than introducing abs() kinks.
  for (const double yawSign : {1., -1.}) {
    for (const double signX : {-1., 1.}) {
      for (const double signY : {-1., 1.}) {
        result(row, 0) = yawSign * signX * mu;
        result(row, 1) = yawSign * signY * mu;
        result(row, 2) = yawSign;
        result(row, 3) = signX * y;
        result(row, 4) = signY * x;
        result(row++, 5) = -mu * (x + y);
      }
    }
  }
  return result;
}

gtsam::NonlinearInequalityConstraints RectangularContactWrench::constraints(
    gtsam::Key key) const {
  const gtsam::Matrix faces = matrix();
  gtsam::NonlinearInequalityConstraints result;
  for (int row = 0; row < faces.rows(); ++row) {
    const gtsam::Matrix16 face = faces.row(row) / faces.row(row).norm();
    const gtsam::JacobianFactor factor(key, face, gtsam::Vector1::Zero());
    result.push_back(
        gtsam::LinearConstraint::LessEqual(factor).createInequalityFactor());
  }
  return result;
}

}  // namespace gtdynamics
