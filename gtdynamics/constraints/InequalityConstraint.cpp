/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  InequalityConstraint.cpp
 * @brief Inequality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#include <gtdynamics/constraints/InequalityConstraint.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtdynamics {

/* <=======================================================================> */
/* <===================== LinearInequalityConstraint ======================> */
/* <=======================================================================> */

/* ************************************************************************* */
bool LinearInequalityConstraint::feasible(const gtsam::VectorValues &x,
                                          double threshold) const {
  for (const double &entry : (*this)(x)) {
    if (entry < -threshold) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
void LinearInequalityConstraint::print(
    const gtsam::KeyFormatter &key_formatter) const {
  createL2Factor()->print("", key_formatter);
}

/* <=======================================================================> */
/* <================= JacobianLinearInequalityConstraint ==================> */
/* <=======================================================================> */

/* ************************************************************************* */
gtsam::JacobianFactor::shared_ptr
JacobianLinearInequalityConstraint::createConstrainedFactor() const {
  auto factor = std::make_shared<gtsam::JacobianFactor>(*factor_);
  auto sigmas = gtsam::Vector::Zero(dim());
  factor->setModel(true, sigmas);
  return factor;
}

/* ************************************************************************* */
gtsam::MultiJacobian JacobianLinearInequalityConstraint::jacobian() const {
  gtsam::MultiJacobian jac;
  size_t start_col = 0;
  gtsam::Matrix jac_mat = factor_->jacobian().first;
  for (auto it = factor_->begin(); it != factor_->end(); it++) {
    size_t dim = factor_->getDim(it);
    jac.addJacobian(*it, jac_mat.middleCols(start_col, dim));
    start_col += dim;
  }
  return jac;
}

/* <=======================================================================> */
/* <======================== InequalityConstraint =========================> */
/* <=======================================================================> */

/* ************************************************************************* */
void InequalityConstraint::print(
    const gtsam::KeyFormatter &key_formatter) const {
  gtsam::PrintKeySet(keys(), name_ + "\t", key_formatter);
}

/* ************************************************************************* */
LinearInequalityConstraint::shared_ptr
InequalityConstraint::linearize(const gtsam::Values &values) const {
  auto nonlinear_factor = createL2Factor();
  auto linear_factor = nonlinear_factor->linearize(values);
  auto jacobian_factor =
      std::static_pointer_cast<gtsam::JacobianFactor>(linear_factor);
  return std::make_shared<JacobianLinearInequalityConstraint>(jacobian_factor);
}

/* <=======================================================================> */
/* <===================== DoubleExpressionInequality ======================> */
/* <=======================================================================> */

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
DoubleExpressionInequality::geq(const gtsam::Double_ &expression,
                                const double &tolerance) {
  return std::make_shared<DoubleExpressionInequality>(expression, tolerance);
}

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
DoubleExpressionInequality::leq(const gtsam::Double_ &expression,
                                const double &tolerance) {
  gtsam::Double_ neg_expr = gtsam::Double_(0.0) - expression;
  return std::make_shared<DoubleExpressionInequality>(neg_expr, tolerance);
}

/* ************************************************************************* */
bool DoubleExpressionInequality::feasible(const gtsam::Values &x) const {
  return expression_.value(x) >= 0;
}

/* ************************************************************************* */
double DoubleExpressionInequality::operator()(const gtsam::Values &x) const {
  return expression_.value(x);
}

/* ************************************************************************* */
double DoubleExpressionInequality::toleranceScaledViolation(
    const gtsam::Values &x) const {
  double error = expression_.value(x);
  if (error >= 0) {
    return 0;
  } else {
    return -error / tolerance_;
  }
}

/* ************************************************************************* */
bool DoubleExpressionInequality::isActive(const gtsam::Values &x) const {
  double error = expression_.value(x);
  return abs(error / tolerance_) < 1e-5;
}

/* ************************************************************************* */
gtsam::MultiJacobian
DoubleExpressionInequality::jacobians(const gtsam::Values &x) const {
  auto keyset = keys();
  gtsam::KeyVector keyvector(keyset.begin(), keyset.end());
  std::vector<gtsam::Matrix> H(keys().size());
  expression_.value(x, H);
  gtsam::MultiJacobian jac;
  for (size_t i = 0; i < keyvector.size(); i++) {
    jac.addJacobian(keyvector.at(i), H.at(i)); // TODO: divide by tolerance?
  }
  return jac;
}

/* ************************************************************************* */
gtsam::NonlinearEqualityConstraint::shared_ptr
DoubleExpressionInequality::createEqualityConstraint() const {
  return std::make_shared<gtsam::ExpressionEqualityConstraint<double>>(
      expression_, 0.0, gtsam::Vector1(tolerance_));
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
DoubleExpressionInequality::createPenaltyFactor(const double mu) const {
  return std::make_shared<gtsam::PenaltyFactor>(createL2Factor(mu), true);
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
DoubleExpressionInequality::createL2Factor(const double mu) const {
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, tolerance_ / sqrt(mu));
  return std::make_shared<gtsam::ExpressionFactor<double>>(noise, 0.0,
                                                           expression_);
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
DoubleExpressionInequality::createSmoothPenaltyFactor(
    const double mu, const double buffer_width) const {
  auto smooth_penalty_function =
      gtsam::SmoothBarrierFunction(0, tolerance_ * buffer_width);
  gtsam::Double_ error(smooth_penalty_function, expression_);
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, tolerance_ / sqrt(mu));
  return std::make_shared<gtsam::ExpressionFactor<double>>(noise, 0.0, error);
}

/* <=======================================================================> */
/* <=================== TwinDoubleExpressionInequality ====================> */
/* <=======================================================================> */

/* ************************************************************************* */
double
TwinDoubleExpressionInequality::operator()(const gtsam::Values &x) const {
  // TODO: this should return a vector
  double eval1 = (*ineq1_)(x);
  double eval2 = (*ineq2_)(x);
  return sqrt(eval1 * eval1 + eval2 * eval2);
}

/* ************************************************************************* */
double TwinDoubleExpressionInequality::toleranceScaledViolation(
    const gtsam::Values &x) const {
  double error1 = ineq1_->toleranceScaledViolation(x);
  double error2 = ineq2_->toleranceScaledViolation(x);
  return sqrt(error1 * error1 + error2 * error2);
}

/* ************************************************************************* */
bool TwinDoubleExpressionInequality::isActive(const gtsam::Values &x) const {
  return ineq1_->isActive(x) || ineq2_->isActive(x);
}

/* ************************************************************************* */
size_t TwinDoubleExpressionInequality::dim() const { return 2; }

/* ************************************************************************* */
gtsam::KeySet TwinDoubleExpressionInequality::keys() const {
  gtsam::KeySet all_keys = ineq1_->keys();
  all_keys.merge(ineq2_->keys());
  return all_keys;
}

/* ************************************************************************* */
gtsam::NonlinearEqualityConstraint::shared_ptr
TwinDoubleExpressionInequality::createEqualityConstraint() const {
  return std::make_shared<gtsam::ExpressionEqualityConstraint<gtsam::Vector2>>(
      expression_, gtsam::Vector2::Zero(), tolerance_);
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
TwinDoubleExpressionInequality::createPenaltyFactor(const double mu) const {
  return std::make_shared<gtsam::PenaltyFactor>(createL2Factor(mu), true);
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
TwinDoubleExpressionInequality::createL2Factor(const double mu) const {
  auto noise = gtsam::noiseModel::Isotropic::Sigmas(1 / sqrt(mu) * tolerance_);
  return std::make_shared<gtsam::ExpressionFactor<gtsam::Vector2>>(
      noise, gtsam::Vector2::Zero(), expression_);
}

/* ************************************************************************* */
gtsam::MultiJacobian
TwinDoubleExpressionInequality::jacobians(const gtsam::Values &x) const {
  auto jac1 = ineq1_->jacobians(x);
  auto jac2 = ineq2_->jacobians(x);
  return gtsam::MultiJacobian::VerticalStack(jac1, jac2);
}

/* ************************************************************************* */
gtsam::NoiseModelFactor::shared_ptr
TwinDoubleExpressionInequality::createSmoothPenaltyFactor(
    const double mu, const double buffer_width) const {
  auto smooth_penalty_function =
      gtsam::SmoothBarrierFunction(0, tolerance_(0) * buffer_width);
  gtsam::Double_ error1(smooth_penalty_function, ineq1_->expression());
  gtsam::Double_ error2(smooth_penalty_function, ineq2_->expression());
  gtsam::Vector2_ error(gtsam::double_stack, error1, error2);
  auto noise = gtsam::noiseModel::Isotropic::Sigmas(1 / sqrt(mu) * tolerance_);
  return std::make_shared<gtsam::ExpressionFactor<gtsam::Vector2>>(
      noise, gtsam::Vector2::Zero(), error);
}

/* ************************************************************************* */
gtsam::Vector2_ TwinDoubleExpressionInequality::ConstructExpression(
    const gtsam::Double_ &expr1, const gtsam::Double_ &expr2) {
  gtsam::Matrix21 H1, H2;
  H1 << 1, 0;
  H2 << 0, 1;
  const std::function<gtsam::Vector2(double)> combine1 = [H1](const double &x) {
    return H1 * x;
  };
  const std::function<gtsam::Vector2(double)> combine2 = [H2](const double &x) {
    return H2 * x;
  };
  gtsam::Vector2_ expr1_v2 = gtsam::linearExpression(combine1, expr1, H1);
  gtsam::Vector2_ expr2_v2 = gtsam::linearExpression(combine2, expr2, H2);
  return expr1_v2 + expr2_v2;
}

/* <=======================================================================> */
/* <======================= InequalityConstraints =========================> */
/* <=======================================================================> */

/* ************************************************************************* */
bool InequalityConstraints::feasible(const gtsam::Values &x) const {
  for (const auto &constraint : *this) {
    if (!constraint->feasible(x))
      return false;
  }
  return true;
}

/* ************************************************************************* */
gtsam::KeySet InequalityConstraints::keys() const {
  gtsam::KeySet keys;
  for (const auto &constraint : *this) {
    keys.merge(constraint->keys());
  }
  return keys;
}

/* ************************************************************************* */
gtsam::VariableIndex InequalityConstraints::varIndex() const {
  gtsam::VariableIndex var_index;
  for (size_t constraint_idx = 0; constraint_idx < size(); constraint_idx++) {
    const auto &constraint = at(constraint_idx);
    var_index.augmentExistingFactor(constraint_idx, constraint->keys());
  }
  return var_index;
}

/* ************************************************************************* */
size_t InequalityConstraints::dim() const {
  size_t dimension = 0;
  for (const auto &constraint : *this) {
    dimension += constraint->dim();
  }
  return dimension;
}

/* ************************************************************************* */
double InequalityConstraints::evaluateViolationL2Norm(
    const gtsam::Values &values) const {
  double violation = 0;
  for (const auto &constraint : *this) {
    violation += pow(constraint->toleranceScaledViolation(values), 2);
  }
  return sqrt(violation);
}

/* ************************************************************************* */
gtsam::NonlinearFactorGraph
InequalityConstraints::meritGraph(const double mu, const bool smooth,
                                  const double buffer_width) const {
  gtsam::NonlinearFactorGraph graph;
  if (smooth) {
    for (const auto &constraint : *this) {
      graph.add(constraint->createSmoothPenaltyFactor(mu, buffer_width));
    }
  } else {
    for (const auto &constraint : *this) {
      graph.add(constraint->createPenaltyFactor(mu));
    }
  }
  return graph;
}

/* ************************************************************************* */
void InequalityConstraints::print(
    const gtsam::KeyFormatter &key_formatter) const {
  for (const auto &constraint : *this) {
    constraint->print(key_formatter);
  }
}

/* <=======================================================================> */
/* <==================== LinearInequalityConstraints ======================> */
/* <=======================================================================> */

/* ************************************************************************* */
gtsam::GaussianFactorGraph LinearInequalityConstraints::constraintGraph(
    const gtsam::IndexSet &active_indices) const {
  gtsam::GaussianFactorGraph graph;
  for (const auto &index : active_indices) {
    graph.push_back(at(index)->createConstrainedFactor());
  }
  return graph;
}

/* ************************************************************************* */
void LinearInequalityConstraints::print(
    const gtsam::KeyFormatter &key_formatter) const {
  for (const auto &constraint : *this) {
    constraint->print(key_formatter);
  }
}

} // namespace gtdynamics
