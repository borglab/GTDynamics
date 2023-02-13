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

#include <gtdynamics/factors/BiasedFactor.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>

namespace gtdynamics {

gtsam::NoiseModelFactor::shared_ptr DoubleExpressionEquality::createFactor(
    const double mu, std::optional<gtsam::Vector> bias) const {
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
gtsam::NoiseModelFactor::shared_ptr FactorZeroErrorConstraint::createFactor(
    const double mu, std::optional<gtsam::Vector> bias) const {
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(tolerance_ / sqrt(mu));
  if (bias) {
    return std::make_shared<gtsam::BiasedFactor>(factor_, *bias, noise);
  }
  return factor_->cloneWithNewNoiseModel(noise);
}

/* ************************************************************************* */
bool FactorZeroErrorConstraint::feasible(const gtsam::Values& x) const {
  auto result = factor_->unwhitenedError(x);
  for (int i = 0; i < dim(); i++) {
    if (abs(result[i]) > tolerance_[i]) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
gtsam::Vector FactorZeroErrorConstraint::operator()(
    const gtsam::Values& x) const {
  return factor_->unwhitenedError(x);
}

/* ************************************************************************* */
gtsam::Vector FactorZeroErrorConstraint::toleranceScaledViolation(
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
    auto noise_factor =
        std::static_pointer_cast<gtsam::NoiseModelFactor>(factor);
    constraints.emplace_shared<FactorZeroErrorConstraint>(noise_factor);
  }
  return constraints;
}

/* ************************************************************************* */
size_t EqualityConstraints::dim() const {
  size_t dimension = 0;
  for (const auto& constraint : *this) {
      dimension += constraint->dim();
  }
  return dimension;
}

}  // namespace gtdynamics
