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
  std::vector<std::pair<size_t, size_t>> constraint_rows_;

public:
  TangentCone(const Matrix &A,
              const std::vector<std::pair<size_t, size_t>> &constraint_rows =
                  std::vector<std::pair<size_t, size_t>>())
      : A_(A), A_cost_(Matrix::Identity(A.cols(), A.cols())),
        constraint_rows_(constraint_rows) {
    if (constraint_rows_.size() == 0) {
      for (size_t i = 0; i < A.rows(); i++) {
        constraint_rows_.emplace_back(i, 1);
      }
    }
  }

  /** Project the vector xi into cone, by solving the i-constrained QP problem:
   *       argmin x     ||x - xi||^2
   *         s.t.       A * x >=0
   * @return Indices of blocking constraints and the projected vector.
   */
  std::pair<IndexSet, Vector> project(const Vector &xi) const;

  const Matrix &A() const { return A_; }

protected:
  /** Compute the solution to the EQP problem defined by the active constraints.
   *       argmin x     ||x - xi||^2
   *         s.t.       A_{active_rows} * x =0
   * @return [x, lambda] representing the solution and Lagrangian multipliers.
   */
  std::pair<Vector, Vector>
  computeUpdate(const std::vector<size_t> &active_indices_vec,
                const Vector &xi) const;

  /** Identify the first inactive row that blocks the update step.
   * @param p update step, e.g., solution to the EQP problem subtracted by
   * prev_x.
   * @param active_indices current active constraint indices.
   * @param prev_x solution to the EQP problem of the previous iteration.
   * @return [alpha, blocking_idx] representing the scale ratio to clip the
   * update step p, and the corresponding blocking constraint index.
   */
  std::pair<double, size_t>
  identifyBlockingConstraint(const Vector &p, const IndexSet &active_indices,
                             const Vector &prev_x) const;
};

} // namespace gtsam
