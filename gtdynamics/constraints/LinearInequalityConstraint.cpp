/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  LinearInequalityConstraint.cpp
 * @brief Linear inequality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#include <gtdynamics/constraints/LinearInequalityConstraint.h>

namespace gtdynamics {

/* <=======================================================================> */
/* <===================== LinearInequalityConstraint ======================> */
/* <=======================================================================> */

/* ************************************************************************* */
bool LinearInequalityConstraint::feasible(const gtsam::VectorValues &x,
                                          double threshold) const {
  for (const double &entry : (*this)(x)) {
    if (entry > threshold) {
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
MultiJacobian JacobianLinearInequalityConstraint::jacobian() const {
  MultiJacobian jac;
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
/* <==================== LinearInequalityConstraints ======================> */
/* <=======================================================================> */

/* ************************************************************************* */
gtsam::GaussianFactorGraph LinearInequalityConstraints::constraintGraph(
    const IndexSet &active_indices) const {
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
