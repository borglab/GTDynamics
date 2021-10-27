/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  EqualityConstraint-inl.h
 * @brief Equality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#pragma once

namespace gtdynamics {

gtsam::NoiseModelFactor::shared_ptr DoubleExpressionEquality::createFactor(
    const double mu, boost::optional<gtsam::Vector&> bias) const {
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, tolerance_ / sqrt(mu));
  double measure = 0.0;
  if (bias) {
    measure = -(*bias)(0);
  }
  return gtsam::NoiseModelFactor::shared_ptr(
      new gtsam::ExpressionFactor<double>(noise, measure, expression_));
}

bool DoubleExpressionEquality::feasible(const gtsam::Values& x) const {
  double result = expression_.value(x);
  return abs(result) <= tolerance_;
}

gtsam::Vector DoubleExpressionEquality::operator()(
    const gtsam::Values& x) const {
  double result = expression_.value(x);
  return (gtsam::Vector(1) << result).finished();
}

gtsam::Vector DoubleExpressionEquality::toleranceScaledViolation(
    const gtsam::Values& x) const {
  double result = expression_.value(x);
  return (gtsam::Vector(1) << result / tolerance_).finished();
}

size_t DoubleExpressionEquality::dim() const { return 1; }

template <int P>
gtsam::NoiseModelFactor::shared_ptr VectorExpressionEquality<P>::createFactor(
    const double mu, boost::optional<gtsam::Vector&> bias) const {
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(tolerance_ / sqrt(mu));
  VectorP measure = VectorP::Zero();
  if (bias) {
    measure = -*bias;
  }
  return gtsam::NoiseModelFactor::shared_ptr(
      new gtsam::ExpressionFactor<VectorP>(noise, measure, expression_));
}

template <int P>
bool VectorExpressionEquality<P>::feasible(const gtsam::Values& x) const {
  VectorP result = expression_.value(x);
  for (int i = 0; i < P; i++) {
    if (abs(result[i]) > tolerance_[i]) {
      return false;
    }
  }
  return true;
}

template <int P>
gtsam::Vector VectorExpressionEquality<P>::operator()(
    const gtsam::Values& x) const {
  return expression_.value(x);
}

template <int P>
gtsam::Vector VectorExpressionEquality<P>::toleranceScaledViolation(
    const gtsam::Values& x) const {
  auto violation = expression_.value(x);
  // TODO(yetong): figure out how to perform element-wise division
  VectorP scaled_violation;
  for (int i = 0; i < P; i++) {
    scaled_violation(i) = violation(i) / tolerance_(i);
  }
  return scaled_violation;
}

template <int P>
size_t VectorExpressionEquality<P>::dim() const {
  return P;
}

}  // namespace gtdynamics