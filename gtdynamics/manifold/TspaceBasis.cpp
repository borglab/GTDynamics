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

namespace gtsam {

/* ************************************************************************* */
TspaceBasis::shared_ptr
TspaceBasis::create(const TspaceBasisParams::shared_ptr params,
                    const ConnectedComponent::shared_ptr cc,
                    const Values &values,
                    std::optional<const KeyVector> basis_keys,
                    std::optional<size_t> manifold_dim) {
  TspaceBasis::shared_ptr basis;

  if (manifold_dim && *manifold_dim == 0) {
    return std::make_shared<EmptyBasis>(params);
  } else if (params->basis_type == BasisType::MATRIX) {
    basis = std::make_shared<MatrixBasis>(params, cc, values);
  } else if (params->basis_type == BasisType::SPARSE_MATRIX) {
    basis = std::make_shared<SparseMatrixBasis>(params, cc, values);
  } else if (params->basis_type == BasisType::SPECIFY_VARIABLES) {
    basis =
        std::make_shared<FixedVarBasis>(params, cc, values, basis_keys);
  }

  // Check the total dimension of basis variables should be the same as the
  // dimension of the manifold.
  if (manifold_dim) {
    if (basis->dim() != *manifold_dim) {
      throw std::runtime_error(
          "specified basis has wrong dimensions. manifold dim: " +
          std::to_string(*manifold_dim) +
          "\tbasis dim: " + std::to_string(basis->dim()));
    }
  }
  return basis;
}

/* ************************************************************************* */
MatrixBasis::MatrixBasis(const TspaceBasisParams::shared_ptr &params,
                         const ConnectedComponent::shared_ptr &cc,
                         const Values &values)
    : TspaceBasis(params) {
  auto linear_graph = cc->merit_graph_.linearize(values);
  JacobianFactor combined(*linear_graph);
  auto augmented = combined.augmentedJacobian();
  Matrix A = augmented.leftCols(augmented.cols() - 1); // m x n
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  basis_ = lu.kernel(); // n x n-m
  size_t position = 0;
  for (const Key &key : combined.keys()) {
    size_t var_dim = values.at(key).dim();
    var_dim_[key] = var_dim;
    var_location_[key] = position;
    position += var_dim;
  }
  total_basis_dim_ = basis_.cols();
  is_constructed_ = true;
}

/* ************************************************************************* */
void MatrixBasis::construct(const ConnectedComponent::shared_ptr &cc,
                            const Values &values) {
  auto linear_graph = cc->merit_graph_.linearize(values);
  JacobianFactor combined(*linear_graph);
  auto augmented = combined.augmentedJacobian();
  Matrix A = augmented.leftCols(augmented.cols() - 1); // m x n
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  basis_ = lu.kernel(); // n x n-m
  is_constructed_ = true;
}

/* ************************************************************************* */
VectorValues MatrixBasis::computeTangentVector(const Vector &xi) const {
  Vector x_xi = basis_ * xi;
  VectorValues delta;
  for (const auto &it : var_location_) {
    const Key &key = it.first;
    delta.insert(key, x_xi.middleRows(it.second, var_dim_.at(key)));
  }
  return delta;
}

/* ************************************************************************* */
Vector MatrixBasis::computeXi(const VectorValues &delta) const {
  Vector x_xi = Vector::Zero(basis_.rows());
  for (const auto &it : var_location_) {
    const Key &key = it.first;
    x_xi.middleRows(it.second, var_dim_.at(key)) = delta.at(key);
  }
  // Vector xi = basis_.colPivHouseholderQr().solve(x_xi);
  Vector xi = basis_.completeOrthogonalDecomposition().pseudoInverse() * x_xi;
  return xi;
}

/* ************************************************************************* */
Matrix MatrixBasis::recoverJacobian(const Key &key) const {
  return basis_.middleRows(var_location_.at(key), var_dim_.at(key));
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
    xi_base.middleRows(var_location_.at(key), var_dim_.at(key)) = it.second;
  }
  Vector xi = basis_pinv * xi_base;
  return xi;
}

/* ************************************************************************* */
SparseMatrixBasis::SparseMatrixBasis(
    const TspaceBasisParams::shared_ptr &params,
    const ConnectedComponent::shared_ptr &cc, const Values &values)
    : TspaceBasis(params) {
  size_t position = 0;
  for (const Key &key : values.keys()) {
    size_t var_dim = values.at(key).dim();
    var_dim_[key] = var_dim;
    var_location_[key] = position;
    position += var_dim;
  }

  total_variable_dim_ = position;
  total_constraint_dim_ = 0;
  for (const auto &factor : cc->merit_graph_) {
    total_constraint_dim_ += factor->dim();
  }
  total_basis_dim_ = total_variable_dim_ - total_constraint_dim_;
  if (params_->always_construct_basis) {
    construct(cc, values);
  }
}

/* ************************************************************************* */
void SparseMatrixBasis::construct(const ConnectedComponent::shared_ptr &cc,
                                  const Values &values) {
  auto linear_graph = cc->merit_graph_.linearize(values);

  std::vector<Triplet> triplet_list;
  size_t row_offset = 0;
  for (size_t factor_idx = 0; factor_idx < linear_graph->size(); factor_idx++) {
    const auto &factor = linear_graph->at(factor_idx);
    setSparseEntries(factor, row_offset, triplet_list);
    row_offset += cc->merit_graph_.at(factor_idx)->dim();
  }
  SpMatrix A(total_constraint_dim_, total_variable_dim_);
  A.setFromTriplets(triplet_list.begin(), triplet_list.end());
  is_constructed_ = true;

  SpMatrix A_t = A.transpose();
  SpMatrix Q;
  auto qr = Eigen::SparseQR<SpMatrix, Eigen::COLAMDOrdering<int>>(A_t);
  Q = qr.matrixQ();

  basis_ = Q.rightCols(total_basis_dim_);
}

/* ************************************************************************* */
VectorValues SparseMatrixBasis::computeTangentVector(const Vector &xi) const {
  Vector x_xi = basis_ * xi;
  VectorValues delta;
  for (const auto &it : var_location_) {
    const Key &key = it.first;
    delta.insert(key, x_xi.middleRows(it.second, var_dim_.at(key)));
  }
  return delta;
}

/* ************************************************************************* */
Vector SparseMatrixBasis::computeXi(const VectorValues &delta) const {
  Vector x_xi = Vector::Zero(basis_.rows());
  for (const auto &it : var_location_) {
    const Key &key = it.first;
    x_xi.middleRows(it.second, var_dim_.at(key)) = delta.at(key);
  }

  auto qr = Eigen::SparseQR<SpMatrix, Eigen::COLAMDOrdering<int>>(basis_);
  Vector xi = qr.solve(x_xi);
  return xi;
}

/* ************************************************************************* */
Matrix SparseMatrixBasis::recoverJacobian(const Key &key) const {
  return basis_.middleRows(var_location_.at(key), var_dim_.at(key));
}

/* ************************************************************************* */
Vector SparseMatrixBasis::localCoordinates(const Values &values,
                                           const Values &values_other) const {
  return Vector::Zero(total_basis_dim_);
}

/* ************************************************************************* */
void SparseMatrixBasis::setMatrix(const Matrix &matrix,
                                  const size_t &row_offset,
                                  const size_t &col_offset,
                                  std::vector<Triplet> &triplet_list) const {

  for (size_t i = 0, nRows = matrix.rows(), nCols = matrix.cols(); i < nRows;
       ++i) {
    for (size_t j = 0; j < nCols; ++j) {
      triplet_list.emplace_back(i + row_offset, j + col_offset, matrix(i, j));
    }
  }
}

/* ************************************************************************* */
void SparseMatrixBasis::setSparseEntries(
    const GaussianFactor::shared_ptr &factor, const size_t row_offset,
    std::vector<Triplet> &triplet_list) const {
  Matrix Ab = factor->augmentedJacobian();
  size_t col_position = 0;
  for (const Key &key : factor->keys()) {
    size_t var_dim = var_dim_.at(key);
    setMatrix(Ab.middleCols(col_position, var_dim), row_offset,
              var_location_.at(key), triplet_list);
    col_position += var_dim;
  }
}

/* ************************************************************************* */
FixedVarBasis::FixedVarBasis(
    const TspaceBasisParams::shared_ptr &params,
    const ConnectedComponent::shared_ptr &cc, const Values &values,
    std::optional<const KeyVector> basis_keys)
    : TspaceBasis(params), basis_keys_(*basis_keys) {
  // Set the location of each basis variable.
  size_t location = 0;
  for (const Key &key : basis_keys_) {
    basis_location_[key] = location;
    location += values.at(key).dim();
  }
  total_basis_dim_ = location;

  // Set dimension of variables.
  for (const Key &key : values.keys()) {
    var_dim_[key] = values.at(key).dim();
  }

  // Partially eliminate all other variables (except for the basis variables) in
  // the merit graph of constraints, and form a bayes net. The bays net
  // represents how other variables depends on the basis variables, e.g.,
  // X_other = B x X_basis.
  auto full_ordering =
      Ordering::ColamdConstrainedLast(cc->merit_graph_, basis_keys_);
  ordering_ = full_ordering;
  for (size_t i = 0; i < basis_keys_.size(); i++) {
    ordering_.pop_back();
  }

  // Compute jacobians w.r.t. basis variables.
  if (params_->always_construct_basis) {
    construct(cc, values);
  }
}

/* ************************************************************************* */
FixedVarBasis::FixedVarBasis(const ConnectedComponent::shared_ptr &cc,
                                   const Values &values,
                                   const FixedVarBasis &other)
    : TspaceBasis(other.params_), basis_keys_(other.basis_keys_),
      ordering_(other.ordering_), total_basis_dim_(other.total_basis_dim_),
      basis_location_(other.basis_location_), var_dim_(other.var_dim_) {
  if (params_->always_construct_basis) {
    construct(cc, values);
  }
}

/* ************************************************************************* */
void FixedVarBasis::construct(const ConnectedComponent::shared_ptr &cc,
                                 const Values &values) {
  auto linear_graph = cc->merit_graph_.linearize(values);
  auto elim_result =
      linear_graph->eliminatePartialSequential(ordering_, EliminateQR);
  auto bayes_net = elim_result.first;
  ComputeBayesNetJacobian(*bayes_net, basis_keys_, var_dim_, jacobians_);
  is_constructed_ = true;
}

/* ************************************************************************* */
VectorValues FixedVarBasis::computeTangentVector(const Vector &xi) const {
  VectorValues delta;
  // Set tangent vector for basis variables to corresponding segment in xi
  for (const Key &key : basis_keys_) {
    delta.insert(key, xi.segment(basis_location_.at(key), var_dim_.at(key)));
  }
  // Compute tangent vector of non-basis variables
  for (const auto &it : jacobians_) {
    const Key &var_key = it.first;
    if (delta.exists(var_key))
      continue;
    Vector v = Vector::Zero(var_dim_.at(var_key));
    for (const auto &jac_it : it.second) {
      const Key &basis_key = jac_it.first;
      v += jac_it.second * delta.at(basis_key);
    }
    delta.insert(var_key, v);
  }
  return delta;
}

/* ************************************************************************* */
Vector FixedVarBasis::computeXi(const VectorValues &delta) const {
  Vector xi = Vector::Zero(total_basis_dim_);
  for (const Key &key : basis_keys_) {
    xi.segment(basis_location_.at(key), var_dim_.at(key)) = delta.at(key);
  }
  return xi;
}

/* ************************************************************************* */
Matrix FixedVarBasis::recoverJacobian(const Key &key) const {
  Matrix H = Matrix::Zero(var_dim_.at(key), total_basis_dim_);
  for (const auto &it : jacobians_.at(key)) {
    const Key &basis_key = it.first;
    H.middleCols(basis_location_.at(basis_key), var_dim_.at(basis_key)) =
        it.second;
  }
  return H;
}

/* ************************************************************************* */
Vector FixedVarBasis::localCoordinates(const Values &values,
                                          const Values &values_other) const {
  Vector xi = Vector::Zero(total_basis_dim_);
  for (const Key &key : basis_keys_) {
    Vector x_xi = values.at(key).localCoordinates_(values_other.at(key));
    xi.segment(basis_location_.at(key), var_dim_.at(key)) = x_xi;
  }
  return xi;
}

} // namespace gtsam
