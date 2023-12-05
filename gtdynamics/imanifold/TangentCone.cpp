/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Tangent Cone.cpp
 * @brief Tangent cone implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/TangentCone.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
/// Return the minimum element of a vector and its index.
std::pair<double, size_t> VectorMinimum(const Vector &vec,
                                        const double &default_min = 0) {
  if (vec.size() == 0) {
    return std::make_pair(default_min, 0);
  }
  double min_element = vec(0);
  size_t min_idx = 0;
  for (size_t i = 1; i < vec.size(); i++) {
    if (vec(i) < min_element) {
      min_element = vec(i);
      min_idx = i;
    }
  }
  return std::make_pair(min_element, min_idx);
}

/* ************************************************************************* */
double VectorMax(const Vector &vec, const size_t start_idx, const size_t dim) {
  double max_val = vec(start_idx);
  for (size_t i = start_idx + 1; i < start_idx + dim; i++) {
    max_val = std::max(max_val, vec(i));
  }
  return max_val;
}

/* ************************************************************************* */
std::pair<IndexSet, Vector> TangentCone::project(const Vector &xi) const {
  Vector x, lambda;
  Vector prev_x = Vector::Zero(xi.size());
  IndexSet active_indices;

  while (true) {
    // Compute update step using active constraints.
    std::vector<size_t> active_indices_vec(active_indices.begin(),
                                           active_indices.end());
    std::tie(x, lambda) = computeUpdate(active_indices_vec, xi);
    Vector p = x - prev_x;

    if (p.norm() < 1e-5) {
      // check lambda
      std::vector<std::pair<size_t, double>> active_constraint_lambda;
      size_t start_row = 0;
      double min_lambda = 0;
      size_t min_lambda_constraint_idx = 0;
      for (const auto &constraint_idx : active_indices_vec) {
        size_t dim = constraint_rows_.at(constraint_idx).second;
        double constraint_lambda = VectorMax(lambda, start_row, dim);
        if (constraint_lambda < min_lambda) {
          min_lambda = constraint_lambda;
          min_lambda_constraint_idx = constraint_idx;
        }
        start_row += dim;
      }

      if (min_lambda >= 0) {
        return std::make_pair(active_indices, x);
      }
      active_indices.erase(active_indices_vec.at(min_lambda_constraint_idx));
    } else {
      // check blocking constraints
      double alpha;
      size_t blocking_index;
      std::tie(alpha, blocking_index) =
          identifyBlockingConstraint(p, active_indices, prev_x);
      if (alpha < 1) {
        active_indices.insert(blocking_index);
        x = prev_x + alpha * p;
      }
    }
    prev_x = x;
  }

  return std::make_pair(active_indices, x);
}

/* ************************************************************************* */
std::pair<Vector, Vector>
TangentCone::computeUpdate(const std::vector<size_t> &active_indices_vec,
                           const Vector &xi) const {
  if (active_indices_vec.size() > 0) {
    size_t num_rows = 0;
    for (const auto &constraint_idx : active_indices_vec) {
      num_rows += constraint_rows_.at(constraint_idx).second;
    }
    Vector b_constraint = Vector::Zero(num_rows);
    Matrix A_constraint = Matrix(num_rows, xi.size());
    size_t i = 0;
    for (const auto &constraint_idx : active_indices_vec) {
      const auto &[start_row, dim] = constraint_rows_.at(constraint_idx);
      A_constraint.middleRows(i, dim) = A_.middleRows(start_row, dim);
      i += dim;
    }
    return SolveEQP(A_cost_, xi, A_constraint, b_constraint);
  } else {
    return std::make_pair(xi, Vector::Zero(0));
  }
}

/* ************************************************************************* */
std::pair<double, size_t>
TangentCone::identifyBlockingConstraint(const Vector &p,
                                        const IndexSet &active_indices,
                                        const Vector &prev_x) const {
  size_t blocking_index;
  double min_alpha = 1;
  for (size_t constraint_idx = 0; constraint_idx < constraint_rows_.size();
       constraint_idx++) {
    if (active_indices.find(constraint_idx) == active_indices.end()) {
      const auto &[start_row, dim] = constraint_rows_.at(constraint_idx);
      for (size_t row_idx = start_row; row_idx < start_row + dim; row_idx++) {
        Vector A_i = A_.row(row_idx);
        double Aip = A_i.dot(p);
        if (Aip < 0) {
          double alpha = -A_i.dot(prev_x) / Aip;
          if (alpha < min_alpha) {
            min_alpha = alpha;
            blocking_index = constraint_idx;
          }
        }
      }
    }
  }
  return std::make_pair(min_alpha, blocking_index);
}

} // namespace gtsam
