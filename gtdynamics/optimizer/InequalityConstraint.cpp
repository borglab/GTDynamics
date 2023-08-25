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

} // namespace gtdynamics
