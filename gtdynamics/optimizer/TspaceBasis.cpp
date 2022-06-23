/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TspaceBasis.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/optimizer/TspaceBasis.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
MatrixBasis::MatrixBasis(const ConnectedComponent::shared_ptr& cc,
                         const Values& values) {
  auto linear_graph = cc->merit_graph_.linearize(values);
  JacobianFactor combined(*linear_graph);
  auto augmented = combined.augmentedJacobian();
  Matrix A = augmented.leftCols(augmented.cols() - 1);  // m x n
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  basis_ = lu.kernel();  // n x n-m
  size_t position = 0;
  for (const Key& key : combined.keys()) {
    size_t var_dim = values.at(key).dim();
    var_dim_[key] = var_dim;
    var_location_[key] = position;
    position += var_dim;
  }
  total_basis_dim_ = basis_.cols();
}

/* ************************************************************************* */
VectorValues MatrixBasis::computeTangentVector(const Vector& xi) const {
  Vector x_xi = basis_ * xi;
  VectorValues delta;
  for (const auto& it : var_location_) {
    const Key& key = it.first;
    delta.insert(key, x_xi.middleRows(it.second, var_dim_.at(key)));
  }
  return delta;
}

/* ************************************************************************* */
Matrix MatrixBasis::recoverJacobian(const Key& key) const {
  return basis_.middleRows(var_location_.at(key), var_dim_.at(key));
}

/* ************************************************************************* */
Vector MatrixBasis::localCoordinates(const Values& values,
                                     const Values& values_other) const {
  Eigen::MatrixXd basis_pinv =
      basis_.completeOrthogonalDecomposition().pseudoInverse();
  VectorValues delta = values.localCoordinates(values_other);
  Vector xi_base = Vector::Zero(values.dim());
  for (const auto& it : delta) {
    const Key& key = it.first;
    xi_base.middleRows(var_location_.at(key), var_dim_.at(key)) = it.second;
  }
  Vector xi = basis_pinv * xi_base;
  return xi;
}

/* ************************************************************************* */
EliminationBasis::EliminationBasis(const ConnectedComponent::shared_ptr& cc,
                                   const Values& values,
                                   const KeyVector& basis_keys)
    : basis_keys_(basis_keys) {
  // Set the location of each basis variable.
  size_t location = 0;
  for (const Key& key : basis_keys_) {
    basis_location_[key] = location;
    location += values.at(key).dim();
  }
  total_basis_dim_ = location;

  // Set dimension of variables.
  for (const Key& key : values.keys()) {
    var_dim_[key] = values.at(key).dim();
  }

  // Partially eliminate all other variables (except for the basis variables) in
  // the merit graph of constraints, and form a bayes net. The bays net
  // represents how other variables depends on the basis variables, e.g.,
  // X_other = B x X_basis.
  auto linear_graph = cc->merit_graph_.linearize(values);
  auto full_ordering =
      Ordering::ColamdConstrainedLast(*linear_graph, basis_keys_);
  Ordering ordering = full_ordering;
  for (size_t i = 0; i < basis_keys_.size(); i++) {
    ordering.pop_back();
  }
  auto elim_result = linear_graph->eliminatePartialSequential(ordering);
  auto bayes_net = elim_result.first;

  // Compute jacobians w.r.t. basis variables.
  ComputeBayesNetJacobian(*bayes_net, basis_keys_, var_dim_, jacobians_);
}

/* ************************************************************************* */
VectorValues EliminationBasis::computeTangentVector(const Vector& xi) const {
  VectorValues delta;
  // Set tangent vector for basis variables to corresponding segment in xi
  for (const Key& key : basis_keys_) {
    delta.insert(key, xi.segment(basis_location_.at(key), var_dim_.at(key)));
  }
  // Compute tangent vector of non-basis variables
  for (const auto& it : jacobians_) {
    const Key& var_key = it.first;
    if (delta.exists(var_key)) continue;
    Vector v = Vector::Zero(var_dim_.at(var_key));
    for (const auto& jac_it : it.second) {
      const Key& basis_key = jac_it.first;
      v += jac_it.second * delta.at(basis_key);
    }
    delta.insert(var_key, v);
  }
  return delta;
}

/* ************************************************************************* */
Matrix EliminationBasis::recoverJacobian(const Key& key) const {
  Matrix H = Matrix::Zero(var_dim_.at(key), total_basis_dim_);
  for (const auto& it : jacobians_.at(key)) {
    const Key& basis_key = it.first;
    H.middleCols(basis_location_.at(basis_key), var_dim_.at(basis_key)) =
        it.second;
  }
  return H;
}

/* ************************************************************************* */
Vector EliminationBasis::localCoordinates(const Values& values,
                                          const Values& values_other) const {
  Vector xi = Vector::Zero(total_basis_dim_);
  for (const Key& key : basis_keys_) {
    Vector x_xi = values.at(key).localCoordinates_(values_other.at(key));
    xi.segment(basis_location_.at(key), var_dim_.at(key)) = x_xi;
  }
  return xi;
}

}  // namespace gtsam
