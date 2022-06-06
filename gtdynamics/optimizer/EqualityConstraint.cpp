/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  EqualityConstraint.cpp
 * @brief Equality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#include <gtdynamics/optimizer/EqualityConstraint.h>

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

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr ZeroErrorFactorEquality::createFactor(
    const double mu, boost::optional<gtsam::Vector&> bias) const {
  if (bias) {
    std::cerr << "Factor Equality not implemented bias yet.\n";
  }
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(tolerance_ / sqrt(mu));
  return factor_->cloneWithNewNoiseModel(noise);
}

/* ************************************************************************* */
bool ZeroErrorFactorEquality::feasible(const gtsam::Values& x) const {
  auto result = factor_->unwhitenedError(x);
  for (int i = 0; i < dim(); i++) {
    if (abs(result[i]) > tolerance_[i]) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
gtsam::Vector ZeroErrorFactorEquality::operator()(const gtsam::Values& x) const {
  return factor_->unwhitenedError(x);
}

/* ************************************************************************* */
gtsam::Vector ZeroErrorFactorEquality::toleranceScaledViolation(
    const gtsam::Values& x) const {
  auto violation = factor_->unwhitenedError(x);
  for (int i = 0; i < dim(); i++) {
    violation(i) = violation(i) / tolerance_(i);
  }
  return violation;
}

/* ************************************************************************* */
EqualityConstraints ConstraintsFromGraph(
    const gtsam::NonlinearFactorGraph& graph) {
  EqualityConstraints constraints;
  for (const auto& factor : graph) {
    auto noise_factor = boost::static_pointer_cast<gtsam::NoiseModelFactor>(factor);
    constraints.emplace_shared<ZeroErrorFactorEquality>(noise_factor);
  }
  return constraints;
}


}  // namespace gtdynamics
