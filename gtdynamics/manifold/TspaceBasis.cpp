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

#include <Eigen/SparseQR>
#include <gtdynamics/manifold/TspaceBasis.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <stdexcept>
#include <utility>

namespace gtsam {

/* ************************************************************************* */
Matrix MatrixBasis::rearrangeMatrix(const Matrix &A, const KeyVector &A_keys) const {
  Matrix jac_mat = Matrix::Zero(A.rows(), attributes_->total_var_dim);
  size_t A_col_idx = 0;
  for (const Key &key : A_keys) {
    const size_t &var_dim = attributes_->var_dim.at(key);
    jac_mat.middleCols(attributes_->var_location.at(key), var_dim) =
        A.middleCols(A_col_idx, var_dim);
    A_col_idx += var_dim;
  }
  return jac_mat;
}

/* ************************************************************************* */
Matrix MatrixBasis::computeConstraintJacobian(const Values &values) const {
  auto linear_graph = attributes_->merit_graph.linearize(values);
  JacobianFactor combined(*linear_graph);
  if (combined.keys().size() == values.keys().size()) {
    return combined.jacobian().first;
  }

  // Treatment for unconstrained keys
  Matrix A = combined.jacobian().first;
  return rearrangeMatrix(A, combined.keys());
}

/* ************************************************************************* */
MatrixBasis::MatrixBasis(const TspaceBasisParams::shared_ptr &params,
                         const EqualityConstraints::shared_ptr &constraints,
                         const Values &values)
    : TspaceBasis(params), attributes_(std::make_shared<Attributes>()) {
  // set attributes
  attributes_->total_constraint_dim = constraints->dim();
  attributes_->total_var_dim = values.dim();
  attributes_->total_basis_dim =
      attributes_->total_var_dim - attributes_->total_constraint_dim;
  attributes_->merit_graph = constraints->meritGraph();
  size_t position = 0;
  for (const Key &key : values.keys()) {
    size_t var_dim = values.at(key).dim();
    attributes_->var_dim[key] = var_dim;
    attributes_->var_location[key] = position;
    position += var_dim;
  }
  if (params_->always_construct_basis) {
    construct(values);
  }
}

/* ************************************************************************* */
void MatrixBasis::construct(const Values &values) {
  if (attributes_->merit_graph.size() == 0) {
    basis_ = Matrix::Identity(attributes_->total_basis_dim,
                              attributes_->total_basis_dim);
  } else {
    Matrix A = computeConstraintJacobian(values); // m x n
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    basis_ = lu.kernel(); // n x n-m
  }
  is_constructed_ = true;
}

/* ************************************************************************* */
TspaceBasis::shared_ptr MatrixBasis::createWithAdditionalConstraints(
    const EqualityConstraints &constraints, const Values &values,
    bool create_from_scartch) const {
  // attributes
  auto new_merit_graph = constraints.meritGraph();

  auto new_attributes = std::make_shared<Attributes>();
  new_attributes->merit_graph = attributes_->merit_graph;
  new_attributes->merit_graph.add(new_merit_graph);
  new_attributes->var_location = attributes_->var_location;
  new_attributes->var_dim = attributes_->var_dim;
  new_attributes->total_var_dim = attributes_->total_var_dim;
  new_attributes->total_constraint_dim = attributes_->total_constraint_dim + constraints.dim();
  new_attributes->total_basis_dim = new_attributes->total_var_dim - new_attributes->total_constraint_dim;

  if (create_from_scartch) {
    if (!is_constructed_) {
      throw std::runtime_error("basis not constructed yet.");
    }
    auto new_linear_graph = new_merit_graph.linearize(values);
    JacobianFactor combined(*new_linear_graph);
    Matrix A_new = combined.jacobian().first;
    A_new = rearrangeMatrix(A_new, combined.keys());
    Matrix AB_new = A_new * basis_;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(AB_new);
    Matrix M = lu.kernel();
    Matrix new_basis = basis_ * M;
    return std::make_shared<MatrixBasis>(params_, new_attributes, new_basis);
  }
  else {
    auto new_basis = std::make_shared<MatrixBasis>(params_, new_attributes);
    new_basis->construct(values);
    return new_basis;
  }
}

/* ************************************************************************* */
VectorValues MatrixBasis::computeTangentVector(const Vector &xi) const {
  VectorValues delta;
  Vector x_xi = basis_ * xi;
  for (const auto &it : attributes_->var_location) {
    const Key &key = it.first;
    delta.insert(key, x_xi.middleRows(it.second, attributes_->var_dim.at(key)));
  }
  return delta;
}

/* ************************************************************************* */
Vector MatrixBasis::computeXi(const VectorValues &delta) const {
  Vector x_xi = Vector::Zero(basis_.rows());
  for (const auto &it : attributes_->var_location) {
    const Key &key = it.first;
    x_xi.middleRows(it.second, attributes_->var_dim.at(key)) = delta.at(key);
  }
  // Vector xi = basis_.colPivHouseholderQr().solve(x_xi);
  return basis_.completeOrthogonalDecomposition().pseudoInverse() * x_xi;
}

/* ************************************************************************* */
Matrix MatrixBasis::recoverJacobian(const Key &key) const {
  return basis_.middleRows(attributes_->var_location.at(key),
                           attributes_->var_dim.at(key));
}

/* ************************************************************************* */
Vector MatrixBasis::localCoordinates(const Values &values,
                                     const Values &values_other) const {
  Eigen::MatrixXd basis_pinv =
      basis_.completeOrthogonalDecomposition().pseudoInverse();
  VectorValues delta = values.localCoordinates(values_other);
  Vector xi_base = Vector::Zero(values.dim());
  for (const auto &it : delta) {
    const Key &key = it.first;
    xi_base.middleRows(attributes_->var_location.at(key),
                       attributes_->var_dim.at(key)) = it.second;
  }
  Vector xi = basis_pinv * xi_base;
  return xi;
}

// /* ************************************************************************* */
// SparseMatrixBasis::SparseMatrixBasis(
//     const TspaceBasisParams::shared_ptr &params,
//     const ConnectedComponent::shared_ptr &cc, const Values &values)
//     : TspaceBasis(params) {
//   size_t position = 0;
//   for (const Key &key : values.keys()) {
//     size_t var_dim = values.at(key).dim();
//     var_dim_[key] = var_dim;
//     var_location_[key] = position;
//     position += var_dim;
//   }

//   total_variable_dim_ = position;
//   total_constraint_dim_ = 0;
//   for (const auto &factor : cc->merit_graph_) {
//     total_constraint_dim_ += factor->dim();
//   }
//   total_basis_dim_ = total_variable_dim_ - total_constraint_dim_;
//   if (params_->always_construct_basis) {
//     construct(cc, values);
//   }
// }

// /* ************************************************************************* */
// void SparseMatrixBasis::construct(const ConnectedComponent::shared_ptr &cc,
//                                   const Values &values) {
//   auto linear_graph = cc->merit_graph_.linearize(values);

//   std::vector<Triplet> triplet_list;
//   size_t row_offset = 0;
//   for (size_t factor_idx = 0; factor_idx < linear_graph->size(); factor_idx++) {
//     const auto &factor = linear_graph->at(factor_idx);
//     setSparseEntries(factor, row_offset, triplet_list);
//     row_offset += cc->merit_graph_.at(factor_idx)->dim();
//   }
//   SpMatrix A(total_constraint_dim_, total_variable_dim_);
//   A.setFromTriplets(triplet_list.begin(), triplet_list.end());
//   is_constructed_ = true;

//   SpMatrix A_t = A.transpose();
//   SpMatrix Q;
//   auto qr = Eigen::SparseQR<SpMatrix, Eigen::COLAMDOrdering<int>>(A_t);
//   Q = qr.matrixQ();

//   basis_ = Q.rightCols(total_basis_dim_);
// }

// /* ************************************************************************* */
// VectorValues SparseMatrixBasis::computeTangentVector(const Vector &xi) const {
//   Vector x_xi = basis_ * xi;
//   VectorValues delta;
//   for (const auto &it : var_location_) {
//     const Key &key = it.first;
//     delta.insert(key, x_xi.middleRows(it.second, var_dim_.at(key)));
//   }
//   return delta;
// }

// /* ************************************************************************* */
// Vector SparseMatrixBasis::computeXi(const VectorValues &delta) const {
//   Vector x_xi = Vector::Zero(basis_.rows());
//   for (const auto &it : var_location_) {
//     const Key &key = it.first;
//     x_xi.middleRows(it.second, var_dim_.at(key)) = delta.at(key);
//   }

//   auto qr = Eigen::SparseQR<SpMatrix, Eigen::COLAMDOrdering<int>>(basis_);
//   Vector xi = qr.solve(x_xi);
//   return xi;
// }

// /* ************************************************************************* */
// Matrix SparseMatrixBasis::recoverJacobian(const Key &key) const {
//   return basis_.middleRows(var_location_.at(key), var_dim_.at(key));
// }

// /* ************************************************************************* */
// Vector SparseMatrixBasis::localCoordinates(const Values &values,
//                                            const Values &values_other) const {
//   return Vector::Zero(total_basis_dim_);
// }

// /* ************************************************************************* */
// void SparseMatrixBasis::setMatrix(const Matrix &matrix,
//                                   const size_t &row_offset,
//                                   const size_t &col_offset,
//                                   std::vector<Triplet> &triplet_list) const {

//   for (size_t i = 0, nRows = matrix.rows(), nCols = matrix.cols(); i < nRows;
//        ++i) {
//     for (size_t j = 0; j < nCols; ++j) {
//       triplet_list.emplace_back(i + row_offset, j + col_offset, matrix(i, j));
//     }
//   }
// }

// /* ************************************************************************* */
// void SparseMatrixBasis::setSparseEntries(
//     const GaussianFactor::shared_ptr &factor, const size_t row_offset,
//     std::vector<Triplet> &triplet_list) const {
//   Matrix Ab = factor->augmentedJacobian();
//   size_t col_position = 0;
//   for (const Key &key : factor->keys()) {
//     size_t var_dim = var_dim_.at(key);
//     setMatrix(Ab.middleCols(col_position, var_dim), row_offset,
//               var_location_.at(key), triplet_list);
//     col_position += var_dim;
//   }
// }

/* ************************************************************************* */
EliminationBasis::EliminationBasis(
    const TspaceBasisParams::shared_ptr &params,
    const EqualityConstraints::shared_ptr &constraints, const Values &values,
    std::optional<const KeyVector> basis_keys)
    : TspaceBasis(params), attributes_(std::make_shared<Attributes>()) {
  // Set the location of each basis variable.
  attributes_->total_basis_dim = values.dim() - constraints->dim();
  attributes_->var_dim = values.dims();
  attributes_->merit_graph = constraints->meritGraph();

  if (basis_keys) {
    attributes_->basis_keys = *basis_keys;
    size_t location = 0;
    for (const Key &key : *basis_keys) {
      attributes_->basis_location.insert({key, location});
      location += values.at(key).dim();
    }
    if (attributes_->total_basis_dim != location) {
      throw std::runtime_error(
          "specified basis keys has wrong dimensions. manifold dim: " +
          std::to_string(attributes_->total_basis_dim) +
          "\tbasis keys dim: " + std::to_string(location));
    }
  }
  else {
    throw std::runtime_error("elimination basis wihthout keys not implemented.");
  }

  // Partially eliminate all other variables (except for the basis variables) in
  // the merit graph of constraints, and form a bayes net. The bays net
  // represents how other variables depends on the basis variables, e.g.,
  // X_other = B x X_basis.
  auto full_ordering = Ordering::ColamdConstrainedLast(attributes_->merit_graph,
                                                       attributes_->basis_keys);
  attributes_->ordering = full_ordering;
  for (size_t i = 0; i < attributes_->basis_keys.size(); i++) {
    attributes_->ordering.pop_back();
  }

  // Compute jacobians w.r.t. basis variables.
  if (params_->always_construct_basis) {
    construct(values);
  }
}

/* ************************************************************************* */
EliminationBasis::EliminationBasis(const Values &values,
                                   const EliminationBasis &other)
    : TspaceBasis(other.params_), attributes_(other.attributes_) {
  if (params_->always_construct_basis) {
    construct(values);
  }
}

/* ************************************************************************* */
void EliminationBasis::construct(const Values &values) {
  // TODO: scenarios with unconstrained keys
  auto linear_graph = attributes_->merit_graph.linearize(values);
  auto elim_result = linear_graph->eliminatePartialSequential(
      attributes_->ordering, EliminateQR);
  auto bayes_net = elim_result.first;
  ComputeBayesNetJacobian(*bayes_net, attributes_->basis_keys,
                          attributes_->var_dim, jacobians_);
  is_constructed_ = true;
}

/* ************************************************************************* */
TspaceBasis::shared_ptr EliminationBasis::createWithAdditionalConstraints(
    const EqualityConstraints &constraints, const Values &values, bool create_from_scratch) const {

  // Identify the keys to eliminate
  NonlinearFactorGraph new_merit_graph = constraints.meritGraph();
  KeyVector new_basis_keys;
  KeySet new_constraint_keys = constraints.keys();
  for (const Key &key : attributes_->basis_keys) {
    if (!new_constraint_keys.exists(key)) {
      new_basis_keys.push_back(key);
    }
  }

  // Identify basis keys locations in xi
  std::map<Key, size_t> new_basis_location;
  size_t location = 0;
  for (const Key &key : new_basis_keys) {
    new_basis_location[key] = location;
    location += attributes_->var_dim.at(key);
  }
  size_t new_basis_dim = location;

  // eliminate on the new factors
  // Ordering new_ordering =
  //     Ordering::ColamdConstrainedLast(linear_graph, basis_keys_);
  // for (size_t i = 0; i < new_basis_keys.size(); i++) {
  //   new_ordering.pop_back();
  // }
  // auto elim_result =
  //   linear_graph.eliminatePartialSequential(new_ordering, EliminateQR);
  // auto bayes_net = elim_result.first;
  // MultiJacobians new_jacobians;
  // ComputeBayesNetJacobian(*bayes_net, new_basis_keys, var_dim_,
  // new_jacobians);
  Ordering new_ordering(new_constraint_keys.begin(), new_constraint_keys.end());

  Ordering total_ordering = attributes_->ordering;
  total_ordering.insert(total_ordering.end(), new_ordering.begin(),
                        new_ordering.end());
  auto new_attributes = std::make_shared<Attributes>();
  new_attributes->merit_graph = attributes_->merit_graph;
  new_attributes->merit_graph.add(new_merit_graph);
  new_attributes->basis_keys = new_basis_keys;
  new_attributes->ordering = total_ordering;
  new_attributes->total_basis_dim = new_basis_dim;
  new_attributes->basis_location = new_basis_location;
  new_attributes->var_dim = attributes_->var_dim;
  auto new_basis = std::make_shared<EliminationBasis>(params_, new_attributes);
  if (create_from_scratch) {
    new_basis->construct(values);
  }
  else {
    MultiJacobians new_jacobians;
    for (const Key &key : new_basis_keys) {
      size_t dim = attributes_->var_dim.at(key);
      new_jacobians.insert({key, MultiJacobian(key, Matrix::Identity(dim, dim))});
    }
    for (const Key &key : new_constraint_keys) {
      new_jacobians.insert({key, MultiJacobian()});
    }
    new_basis->jacobians_ = JacobiansMultiply(jacobians_, new_jacobians);
    new_basis->is_constructed_ = true;
  }
  return new_basis;
}

/* ************************************************************************* */
VectorValues EliminationBasis::computeTangentVector(const Vector &xi) const {
  VectorValues delta;
  // Set tangent vector for basis variables to corresponding segment in xi
  for (const Key &key : attributes_->basis_keys) {
    delta.insert(key, xi.segment(attributes_->basis_location.at(key),
                                 attributes_->var_dim.at(key)));
  }
  // Compute tangent vector of non-basis variables
  for (const auto &it : jacobians_) {
    const Key &var_key = it.first;
    if (delta.exists(var_key))
      continue;
    Vector v = Vector::Zero(attributes_->var_dim.at(var_key));
    for (const auto &jac_it : it.second) {
      const Key &basis_key = jac_it.first;
      v += jac_it.second * delta.at(basis_key);
    }
    delta.insert(var_key, v);
  }
  return delta;
}

/* ************************************************************************* */
Vector EliminationBasis::computeXi(const VectorValues &delta) const {
  Vector xi = Vector::Zero(attributes_->total_basis_dim);
  for (const Key &key : attributes_->basis_keys) {
    xi.segment(attributes_->basis_location.at(key),
               attributes_->var_dim.at(key)) = delta.at(key);
  }
  return xi;
}

/* ************************************************************************* */
Matrix EliminationBasis::recoverJacobian(const Key &key) const {
  Matrix H =
      Matrix::Zero(attributes_->var_dim.at(key), attributes_->total_basis_dim);
  for (const auto &it : jacobians_.at(key)) {
    const Key &basis_key = it.first;
    H.middleCols(attributes_->basis_location.at(basis_key),
                 attributes_->var_dim.at(basis_key)) = it.second;
  }
  return H;
}

/* ************************************************************************* */
Vector EliminationBasis::localCoordinates(const Values &values,
                                          const Values &values_other) const {
  Vector xi = Vector::Zero(attributes_->total_basis_dim);
  for (const Key &key : attributes_->basis_keys) {
    Vector x_xi = values.at(key).localCoordinates_(values_other.at(key));
    xi.segment(attributes_->basis_location.at(key),
               attributes_->var_dim.at(key)) = x_xi;
  }
  return xi;
}

} // namespace gtsam
