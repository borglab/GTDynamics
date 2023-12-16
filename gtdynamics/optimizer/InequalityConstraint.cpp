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

#include <gtdynamics/optimizer/InequalityConstraint.h>

namespace gtdynamics {

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
InequalityConstraints::meritGraph(const double mu) const {
  gtsam::NonlinearFactorGraph graph;
  for (const auto &constraint : *this) {
    graph.add(constraint->createBarrierFactor(mu));
  }
  return graph;
}

} // namespace gtdynamics
