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

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
Values ConstraintManifold::constructValues(
    const ConnectedComponent::shared_ptr cc, const gtsam::Values &values,
    const Retractor::shared_ptr &retractor, bool retract_init) {
  Values cm_values;
  for (const gtsam::Key &key : cc->keys_) {
    cm_values.insert(key, values.at(key));
  }
  if (retract_init) {
    return retractor->retractConstraints(std::move(cm_values));
  } else {
    return cm_values;
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
Retractor::shared_ptr ConstraintManifold::constructRetractor(
    const Params::shared_ptr &params,
    const ConnectedComponent::shared_ptr &cc) {
  if (params->retract_params->use_basis_keys) {
    if (params->basis_key_func == NULL) {
      throw std::runtime_error("Basis Key Function not provided for retractor");
    }
    KeyVector basis_keys = params->basis_key_func(cc);
    return Retractor::create( params->retract_params, cc,
                             basis_keys);
  }
  return Retractor::create( params->retract_params, cc);
}

/* ************************************************************************* */
TspaceBasis::shared_ptr ConstraintManifold::constructTspaceBasis(
    const Params::shared_ptr &params, const ConnectedComponent::shared_ptr &cc,
    const Values &values, size_t manifold_dim) {
  if (params->basis_params->use_basis_keys) {
    if (params->basis_key_func == NULL) {
      throw std::runtime_error(
          "Basis Key Function not provided for tspace basis");
    }
    KeyVector basis_keys = params->basis_key_func(cc);
    return TspaceBasis::create(params->basis_params, cc,
                               values, basis_keys, manifold_dim);
  }
  return TspaceBasis::create(params->basis_params, cc,
                             values, boost::none, manifold_dim);
}

/* ************************************************************************* */
const Values ConstraintManifold::feasibleValues() const {
  LevenbergMarquardtOptimizer optimizer(cc_->merit_graph_, values_);
  return optimizer.optimize();
}

} // namespace gtsam
