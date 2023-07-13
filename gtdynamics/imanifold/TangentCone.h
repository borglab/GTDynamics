/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TangentCone.h
 * @brief Tangent cone formed by linearized active inequality constraints
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/optimizer/QPSolver.h>

namespace gtsam {

/** A hyper cone that represents the space defined by A*x >= 0. */
class TangentCone {
public:
  typedef std::shared_ptr<TangentCone> shared_ptr;

protected:
  Matrix A_;
  Matrix A_cost_;

public:
  TangentCone(const Matrix &A)
      : A_(A), A_cost_(Matrix::Identity(A.cols(), A.cols())) {}

  /// Project the vector xi into cone, return blocking constraints and the
  /// projected vector.
  std::pair<IndexSet, Vector> project(const Vector &xi) const {
    Vector x, lambda;
    Vector prev_x = Vector::Zero(xi.size());
    IndexSet active_indices;

    while (true) {
      // Compute update step using active constraints.
      std::vector<size_t> active_indices_vec(active_indices.begin(),
                                             active_indices.end());
      computeUpdate(active_indices_vec, xi, x, lambda);
      Vector p = x - prev_x;

      if (p.norm() < 1e-5) {
        // check lambda
        double min_lambda = 0;
        size_t min_idx;
        std::tie(min_lambda, min_idx) = VectorMinimum(lambda);
        if (active_indices.size() == 0 || min_lambda >= 0) {
          return std::make_pair(active_indices, x);
        } else {
          active_indices.erase(active_indices_vec.at(min_idx));
        }
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

  const Matrix& A() const {return A_; }

protected:
  /// Compute the solution to the EQP problem using active constraints.
  void computeUpdate(const std::vector<size_t> &active_indices_vec,
                     const Vector &xi, Vector &x, Vector &lambda) const {
    if (active_indices_vec.size() > 0) {
      Vector b_constraint = Vector::Zero(active_indices_vec.size());
      Matrix A_constraint = Matrix(active_indices_vec.size(), xi.size());
      for (size_t i = 0; i < active_indices_vec.size(); i++) {
        A_constraint.row(i) = A_.row(active_indices_vec.at(i));
      }
      std::tie(x, lambda) = SolveEQP(A_cost_, xi, A_constraint, b_constraint);
    } else {
      x = xi;
      lambda = Vector::Zero(0);
    }
  }

  /// Return the minimum element of a vector and its index.
  static std::pair<double, size_t>
  VectorMinimum(const Vector &vec, const double &default_min = 0) {
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

  /// Identify the first inactive constraints that blocks the update step.
  std::pair<double, size_t>
  identifyBlockingConstraint(const Vector &p, const IndexSet &active_indices,
                             const Vector &prev_x) const {
    size_t blocking_index;
    double min_alpha = 1;
    for (size_t constraint_idx = 0; constraint_idx < A_.rows();
         constraint_idx++) {
      if (active_indices.find(constraint_idx) == active_indices.end()) {
        Vector A_i = A_.row(constraint_idx);
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
    return std::make_pair(min_alpha, blocking_index);
  }
};

} // namespace gtsam
