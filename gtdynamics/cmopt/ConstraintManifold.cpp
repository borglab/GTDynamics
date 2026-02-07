/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstraintManifold.cpp
 * @brief Constraint manifold implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtsam/base/types.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
Values ConstraintManifold::constructValues(
    const gtsam::Values &values,
    const Retractor::shared_ptr &retractor, bool retract_init) {
  if (retract_init) {
    return retractor->retractConstraints(std::move(values));
  } else {
    return values;
  }
}

/* ************************************************************************* */
const gtsam::Value &ConstraintManifold::recover(const gtsam::Key key,
                                                ChartJacobian H) const {
  if (H) {
    // choose the corresponding rows in basis
    makeSureBasisConstructed();
    *H = basis_->recoverJacobian(key);
  }
  return values_.at(key);
}

/* ************************************************************************* */
ConstraintManifold ConstraintManifold::retract(const gtsam::Vector &xi,
                                               ChartJacobian H1,
                                               ChartJacobian H2) const {
  // Compute delta for each variable and perform update.
  makeSureBasisConstructed();
  // std::cout << "xi: " << xi.transpose() << "\n";
  VectorValues delta = basis_->computeTangentVector(xi);
  Values new_values = retractor_->retract(values_, delta);

  // Set jacobian as 0 since they are not used for optimization.
  if (H1)
    throw std::runtime_error(
        "ConstraintManifold retract jacobian not implemented.");
  if (H2)
    throw std::runtime_error(
        "ConstraintManifold retract jacobian not implemented.");

  // Satisfy the constraints in the connected component.
  return createWithNewValues(new_values);
}

/* ************************************************************************* */
gtsam::Vector ConstraintManifold::localCoordinates(const ConstraintManifold &g,
                                                   ChartJacobian H1,
                                                   ChartJacobian H2) const {
  makeSureBasisConstructed();
  Vector xi = basis_->localCoordinates(values_, g.values_);

  // Set jacobian as 0 since they are not used for optimization.
  if (H1)
    throw std::runtime_error(
        "ConstraintManifold localCoordinates jacobian not implemented.");
  if (H2)
    throw std::runtime_error(
        "ConstraintManifold localCoordinates jacobian not implemented.");

  return xi;
}

/* ************************************************************************* */
void ConstraintManifold::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << "ConstraintManifold" << std::endl;
  values_.print();
}

/* ************************************************************************* */
bool ConstraintManifold::equals(const ConstraintManifold &other,
                                double tol) const {
  return values_.equals(other.values_, tol);
}

/* ************************************************************************* */
const Values ConstraintManifold::feasibleValues() const {
  LevenbergMarquardtOptimizer optimizer(constraints_->meritGraph(), values_);
  return optimizer.optimize();
}

/* ************************************************************************* */
Values EManifoldValues::baseValues() const {
  Values base_values;
  for (const auto &it : *this) {
    base_values.insert(it.second.values());
  }
  return base_values;
}

/* ************************************************************************* */
KeyVector EManifoldValues::keys() const {
  KeyVector key_vector;
  for (const auto &it : *this) {
    key_vector.push_back(it.first);
  }
  return key_vector;
}

/* ************************************************************************* */
VectorValues
EManifoldValues::computeTangentVector(const VectorValues &delta) const {
  VectorValues tangent_vector;
  for (const auto &it : *this) {
    tangent_vector.insert(
        it.second.basis()->computeTangentVector(delta.at(it.first)));
  }
  return tangent_vector;
}

/* ************************************************************************* */
EManifoldValues EManifoldValues::retract(const VectorValues &delta) const {
  EManifoldValues new_values;
  for (const auto &it : *this) {
    new_values.insert({it.first, it.second.retract(delta.at(it.first))});
  }
  return new_values;
}

/* ************************************************************************* */
std::map<Key, size_t> EManifoldValues::dims() const {
  std::map<Key, size_t> dims_map;
  for (const auto &it : *this) {
    dims_map.insert({it.first, it.second.dim()});
  }
  return dims_map;
}

} // namespace gtsam
