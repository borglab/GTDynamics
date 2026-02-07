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
#if defined(GTDYNAMICS_WITH_SUITESPARSE)
#include <SuiteSparseQR.hpp>
#include <cholmod.h>
#endif
#include <gtdynamics/cmopt/TspaceBasis.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <stdexcept>
#include <utility>

namespace gtdynamics {

/* ************************************************************************* */
std::vector<VectorValues> TspaceBasis::basisVectors() const {
  std::vector<VectorValues> basis_vectors;
  basis_vectors.reserve(dim());
  for (size_t i = 0; i < dim(); i++) {
    Vector xi = Vector::Zero(dim());
    xi(i) = 1;
    basis_vectors.emplace_back(computeTangentVector(xi));
  }
  return basis_vectors;
}

/* ************************************************************************* */
Matrix OrthonormalBasis::rearrangeMatrix(const Matrix &A,
                                         const KeyVector &A_keys) const {
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
Matrix OrthonormalBasis::computeConstraintJacobian(const Values &values) const {
  auto linear_graph = attributes_->merit_graph.linearize(values);
  gtsam::JacobianFactor combined(*linear_graph);
  if (combined.keys().size() == values.keys().size()) {
    return combined.jacobian().first;
  }

  // Treatment for unconstrained keys
  Matrix A = combined.jacobian().first;
  return rearrangeMatrix(A, combined.keys());
}

/* ************************************************************************* */
OrthonormalBasis::OrthonormalBasis(
    const EqualityConstraints::shared_ptr &constraints, const Values &values,
    const TspaceBasisParams::shared_ptr &params)
    : TspaceBasis(params), attributes_(std::make_shared<Attributes>()) {
  // set attributes
  attributes_->total_constraint_dim = constraints->dim();
  attributes_->total_var_dim = values.dim();
  attributes_->total_basis_dim =
      attributes_->total_var_dim - attributes_->total_constraint_dim;
  attributes_->merit_graph = constraints->penaltyGraph();
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
void OrthonormalBasis::construct(const Values &values) {
  if (attributes_->merit_graph.size() == 0) {
    basis_ = Matrix::Identity(attributes_->total_basis_dim,
                              attributes_->total_basis_dim);
  } else {
    if (params_->use_sparse) {
      constructSparse(values);
    } else {
      constructDense(values);
    }
  }
  is_constructed_ = true;
}

/* ************************************************************************* */
void OrthonormalBasis::constructDense(const Values &values) {
  Matrix A = computeConstraintJacobian(values); // m x n
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  basis_ = lu.kernel(); // n x n-m
}

/* ************************************************************************* */
void OrthonormalBasis::constructSparse(const Values &values) {
  auto linear_graph = attributes_->merit_graph.linearize(values);
  Ordering ordering;
  auto triplets = linear_graph->sparseJacobian();
  size_t nrows = attributes_->total_constraint_dim;
  size_t ncols = attributes_->total_var_dim + 1;

#if defined(GTDYNAMICS_WITH_SUITESPARSE)
  cholmod_common common;
  cholmod_common *cc = &common;
  cholmod_l_start(cc);

  cholmod_sparse *A_t = SparseJacobianTranspose(nrows, ncols, triplets, cc);
  SuiteSparseQR_factorization<double> *QR = SuiteSparseQR_factorize<double>(
      SPQR_ORDERING_DEFAULT, SPQR_DEFAULT_TOL, A_t, cc);

  cholmod_sparse *selection_mat = LastColsSelectionMat(
      attributes_->total_var_dim, attributes_->total_basis_dim, cc);

  cholmod_sparse *basis_cholmod =
      SuiteSparseQR_qmult(SPQR_QX, QR, selection_mat, cc);

  basis_ = CholmodToEigen(basis_cholmod, cc);

  cholmod_l_free_sparse(&A_t, cc);
  SuiteSparseQR_free(&QR, cc);
  cholmod_l_free_sparse(&selection_mat, cc);
  cholmod_l_free_sparse(&basis_cholmod, cc);
  cholmod_l_finish(cc);
#else
  SpMatrix A_t = SparseJacobianTranspose(nrows, ncols, triplets);
  Eigen::SparseQR<SpMatrix, Eigen::COLAMDOrdering<int>> qr;
  qr.compute(A_t);
  if (qr.info() != Eigen::Success) {
    throw std::runtime_error("Eigen SparseQR failed for tangent basis.");
  }

  SpMatrix selection_mat = LastColsSelectionMat(attributes_->total_var_dim,
                                                attributes_->total_basis_dim);
  basis_ = qr.matrixQ() * Matrix(selection_mat);
#endif
}

/* ************************************************************************* */
TspaceBasis::shared_ptr OrthonormalBasis::createWithAdditionalConstraints(
    const EqualityConstraints &constraints, const Values &values,
    bool create_from_scratch) const {
  // attributes
  auto new_merit_graph = constraints.penaltyGraph();

  auto new_attributes = std::make_shared<Attributes>();
  new_attributes->merit_graph = attributes_->merit_graph;
  new_attributes->merit_graph.add(new_merit_graph);
  new_attributes->var_location = attributes_->var_location;
  new_attributes->var_dim = attributes_->var_dim;
  new_attributes->total_var_dim = attributes_->total_var_dim;
  new_attributes->total_constraint_dim =
      attributes_->total_constraint_dim + constraints.dim();
  new_attributes->total_basis_dim =
      new_attributes->total_var_dim - new_attributes->total_constraint_dim;

  if (create_from_scratch) {
    auto new_basis =
        std::make_shared<OrthonormalBasis>(params_, new_attributes);
    new_basis->construct(values);
    return new_basis;
  }
  if (!is_constructed_) {
    throw std::runtime_error("basis not constructed yet.");
  }
  auto new_linear_graph = new_merit_graph.linearize(values);

  if (params_->use_sparse) {
    return createWithAdditionalConstraintsSparse(*new_linear_graph,
                                                 new_attributes);
  } else {
    return createWithAdditionalConstraintsDense(*new_linear_graph,
                                                new_attributes);
  }
}

/* ************************************************************************* */
TspaceBasis::shared_ptr OrthonormalBasis::createWithAdditionalConstraintsDense(
    const GaussianFactorGraph &graph,
    const Attributes::shared_ptr &new_attributes) const {
  gtsam::JacobianFactor combined(graph);
  Matrix A_new = combined.jacobian().first;
  A_new = rearrangeMatrix(A_new, combined.keys());
  Matrix AB_new = A_new * basis_;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(AB_new);
  Matrix M = lu.kernel();
  Matrix new_basis = basis_ * M;
  return std::make_shared<OrthonormalBasis>(params_, new_attributes, new_basis);
}

/* ************************************************************************* */
TspaceBasis::shared_ptr OrthonormalBasis::createWithAdditionalConstraintsSparse(
    const GaussianFactorGraph &graph,
    const Attributes::shared_ptr &new_attributes) const {
  SpMatrix A_new = eigenSparseJacobian(graph);
  Matrix AB_new = A_new * basis_;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(AB_new);
  Matrix M = lu.kernel();
  Matrix new_basis = basis_ * M;
  return std::make_shared<OrthonormalBasis>(params_, new_attributes, new_basis);
}

/* ************************************************************************* */
VectorValues OrthonormalBasis::computeTangentVector(const Vector &xi) const {
  VectorValues delta;
  Vector x_xi = basis_ * xi;
  for (const auto &[key, location] : attributes_->var_location) {
    delta.insert(key, x_xi.middleRows(location, attributes_->var_dim.at(key)));
  }
  return delta;
}

/* ************************************************************************* */
Vector OrthonormalBasis::computeXi(const VectorValues &delta) const {
  Vector x_xi = Vector::Zero(basis_.rows());
  for (const auto &it : attributes_->var_location) {
    const Key &key = it.first;
    x_xi.middleRows(it.second, attributes_->var_dim.at(key)) = delta.at(key);
  }
  // Vector xi = basis_.colPivHouseholderQr().solve(x_xi);
  return basis_.completeOrthogonalDecomposition().pseudoInverse() * x_xi;
}

/* ************************************************************************* */
Matrix OrthonormalBasis::recoverJacobian(const Key &key) const {
  return basis_.middleRows(attributes_->var_location.at(key),
                           attributes_->var_dim.at(key));
}

/* ************************************************************************* */
Vector OrthonormalBasis::localCoordinates(const Values &values,
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

#if defined(GTDYNAMICS_WITH_SUITESPARSE)
void PrintDense(const cholmod_dense *A, const cholmod_common *cc) {
  for (int i = 0; i < A->nrow; i++) {
    for (int j = 0; j < A->ncol; j++) {
      std::cout << ((double *)(A->x))[j * A->nrow + i] << "\t";
    }
    std::cout << "\n";
  }
}

void PrintSparse(cholmod_sparse *A, cholmod_common *cc) {
  cholmod_dense *A_dense = cholmod_l_sparse_to_dense(A, cc);
  PrintDense(A_dense, cc);
  cholmod_l_free_dense(&A_dense, cc);
}
#endif

/* ************************************************************************* */
#if defined(GTDYNAMICS_WITH_SUITESPARSE)
cholmod_sparse *OrthonormalBasis::SparseJacobianTranspose(
    const size_t nrows, const size_t ncols,
    const std::vector<std::tuple<int, int, double>> &triplets,
    cholmod_common *cc) {
  int A_stype = 0;
  int A_xdtype = CHOLMOD_DOUBLE + CHOLMOD_REAL;

  // count entires not from last col
  size_t nnz = 0;
  size_t last_col = ncols - 1;
  for (const auto &it : triplets) {
    if (std::get<1>(it) < last_col) {
      nnz += 1;
    }
  }

  cholmod_triplet *T =
      cholmod_l_allocate_triplet(ncols - 1, nrows, nnz, A_stype, A_xdtype, cc);
  T->nnz = nnz;
  size_t idx = 0;
  for (const auto &triplet : triplets) {
    if (std::get<1>(triplet) < last_col) {
      std::tie(((int64_t *)(T->j))[idx], ((int64_t *)(T->i))[idx],
               ((double *)(T->x))[idx]) = triplet;
      idx += 1;
    }
  }

  cholmod_sparse *A = (cholmod_sparse *)cholmod_l_triplet_to_sparse(T, nnz, cc);
  cholmod_l_free_triplet(&T, cc);
  return A;
}

/* ************************************************************************* */
cholmod_sparse *OrthonormalBasis::LastColsSelectionMat(const size_t nrows,
                                                       const size_t ncols,
                                                       cholmod_common *cc) {
  int A_stype = 0;
  int A_xdtype = CHOLMOD_DOUBLE + CHOLMOD_REAL;
  size_t nnz = ncols;
  cholmod_triplet *T =
      cholmod_l_allocate_triplet(nrows, nnz, ncols, A_stype, A_xdtype, cc);
  T->nnz = nnz;
  size_t pad = nrows - ncols;
  for (int idx = 0; idx < ncols; idx++) {
    ((int64_t *)(T->i))[idx] = pad + idx;
    ((int64_t *)(T->j))[idx] = idx;
    ((double *)(T->x))[idx] = 1;
  }
  cholmod_sparse *A = (cholmod_sparse *)cholmod_l_triplet_to_sparse(T, nnz, cc);
  cholmod_l_free_triplet(&T, cc);
  // PrintSparse(A, cc);
  return A;
}

/* ************************************************************************* */
OrthonormalBasis::SpMatrix
OrthonormalBasis::CholmodToEigen(cholmod_sparse *A, cholmod_common *cc) {

  cholmod_triplet *T = cholmod_l_sparse_to_triplet(A, cc);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(T->nnz);
  for (int idx = 0; idx < T->nnz; idx++) {
    triplets.emplace_back(((int64_t *)(T->i))[idx], ((int64_t *)(T->j))[idx],
                          ((double *)(T->x))[idx]);
  }

  SpMatrix A_eigen(A->nrow, A->ncol);
  A_eigen.setFromTriplets(triplets.begin(), triplets.end());
  cholmod_l_free_triplet(&T, cc);
  return A_eigen;
}
#else
OrthonormalBasis::SpMatrix OrthonormalBasis::SparseJacobianTranspose(
    const size_t nrows, const size_t ncols,
    const std::vector<std::tuple<int, int, double>> &triplets) {
  const size_t last_col = ncols - 1;
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(triplets.size());
  for (const auto &triplet : triplets) {
    if (std::get<1>(triplet) < last_col) {
      entries.emplace_back(std::get<1>(triplet), std::get<0>(triplet),
                           std::get<2>(triplet));
    }
  }

  SpMatrix A_t(ncols - 1, nrows);
  A_t.setFromTriplets(entries.begin(), entries.end());
  return A_t;
}

OrthonormalBasis::SpMatrix
OrthonormalBasis::LastColsSelectionMat(const size_t nrows, const size_t ncols) {
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(ncols);
  const size_t pad = nrows - ncols;
  for (size_t idx = 0; idx < ncols; idx++) {
    entries.emplace_back(pad + idx, idx, 1.0);
  }

  SpMatrix A(nrows, ncols);
  A.setFromTriplets(entries.begin(), entries.end());
  return A;
}
#endif

/* ************************************************************************* */
OrthonormalBasis::SpMatrix
OrthonormalBasis::EigenSparseJacobian(const GaussianFactorGraph &graph) {
  Ordering ordering(graph.keys());
  size_t rows, cols;
  auto triplets = graph.sparseJacobian(ordering, rows, cols);
  size_t last_col = cols - 1;
  std::vector<Eigen::Triplet<double>> entries;
  for (auto &v : triplets) {
    if (std::get<1>(v) < last_col) {
      entries.emplace_back(std::get<0>(v), std::get<1>(v), std::get<2>(v));
    }
  }
  OrthonormalBasis::SpMatrix A(rows, cols - 1);
  A.setFromTriplets(entries.begin(), entries.end());
  return A;
}

/* ************************************************************************* */
OrthonormalBasis::SpMatrix
OrthonormalBasis::eigenSparseJacobian(const GaussianFactorGraph &graph) const {

  std::vector<Eigen::Triplet<double>> entries;

  size_t nrows = 0;
  for (const auto &factor : graph) {
    Matrix f_jacobian = factor->jacobian().first;
    size_t factor_dim = f_jacobian.rows();
    size_t start_col_f = 0;
    for (const Key &key : *factor) {
      const auto &start_col_jacobian = attributes_->var_location.at(key);
      for (size_t i = 0; i < factor_dim; i++)
        for (size_t j = 0; j < attributes_->var_dim.at(key); j++) {
          const double &s = f_jacobian(i, j + start_col_f);
          if (std::abs(s) > 1e-12)
            entries.emplace_back(nrows + i, start_col_jacobian + j, s);
        }
      start_col_f += attributes_->var_dim.at(key);
    }
    nrows += factor_dim;
  }
  OrthonormalBasis::SpMatrix A(nrows, attributes_->total_var_dim);
  A.setFromTriplets(entries.begin(), entries.end());
  return A;
}

/* ************************************************************************* */
EliminationBasis::EliminationBasis(
    const EqualityConstraints::shared_ptr &constraints, const Values &values,
    const TspaceBasisParams::shared_ptr &params,
    std::optional<const KeyVector> basis_keys)
    : TspaceBasis(params), attributes_(std::make_shared<Attributes>()) {
  // Set the location of each basis variable.
  attributes_->total_basis_dim = values.dim() - constraints->dim();
  attributes_->var_dim = values.dims();
  attributes_->merit_graph = constraints->penaltyGraph();

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
  } else {
    throw std::runtime_error(
        "elimination basis wihthout keys not implemented.");
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
      attributes_->ordering, gtsam::EliminateQR);
  auto bayes_net = elim_result.first;
  ComputeBayesNetJacobian(*bayes_net, attributes_->basis_keys,
                          attributes_->var_dim, jacobians_);
  is_constructed_ = true;
}

/* ************************************************************************* */
TspaceBasis::shared_ptr EliminationBasis::createWithAdditionalConstraints(
    const EqualityConstraints &constraints, const Values &values,
    bool create_from_scratch) const {

  // Identify the keys to eliminate
  NonlinearFactorGraph new_merit_graph = constraints.penaltyGraph();
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
  } else {
    MultiJacobians new_jacobians;
    for (const Key &key : new_basis_keys) {
      size_t dim = attributes_->var_dim.at(key);
      new_jacobians.insert(
          {key, MultiJacobian(key, Matrix::Identity(dim, dim))});
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

} // namespace gtdynamics
