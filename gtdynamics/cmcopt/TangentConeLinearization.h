/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file TangentConeLinearization.h
 * @brief Keep CMC face selection local to component tangent coordinates.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtdynamics/cmcopt/ComponentTangentSpace.h>
#include <gtdynamics/constraints/LinearInequalityConstraint.h>

namespace gtdynamics {
namespace internal {
/** Cached homogeneous inequalities, each belonging to one manifold component.
 * Rows use the code convention A_i delta_i <= 0. Original block indices are
 * retained for nonlinear retraction; QP refinement selects individual rows.
 * The global objective is coupled, but every working-face basis and dual
 * calculation is component-local. Only soft costs enter the global solver.
 */
class TangentConeLinearization {
 public:
  /** A bounded working-set refinement returns its best feasible point.
   * A result is usable as an LM solution only when converged is true.
   */
  struct Result {
    gtsam::VectorValues delta;
    IndexSet blockingIndices;
    size_t numSolves = 0;
    bool converged = false;
  };

  /** Cache unary zero-RHS constraint matrices and their whitened versions.
   * Nonlocal or affine constraints are an assembly error and throw
   * invalid_argument: constraint-connected components must already be built.
   */
  explicit TangentConeLinearization(
      const LinearInequalityConstraints& constraints);

  /// Whether there are any active inequality blocks at this nonlinear state.
  bool empty() const { return blocks_.empty(); }

  /// Construct local bases for the selected whole constraint blocks.
  ComponentTangentSpace workingSpace(const IndexSet& blockingIndices) const;

  /// Return blocks violating a directional inequality by more than tolerance.
  IndexSet violated(const gtsam::VectorValues& delta,
                    double tolerance = 1e-5) const;

  /// Check selected whole blocks are zero in their original whitened metric.
  bool active(const gtsam::VectorValues& delta, const IndexSet& indices,
              double tolerance = 1e-5) const;

  /** Refine a feasible warm start with primal active-set steps.
   * Each quadratic subproblem uses local kernels and an unconstrained global
   * solve. Boundary line search adds rows; local KKT multipliers release rows.
   * Preserve the existing 1e-7 feasibility/step/multiplier tolerances.
   * Dependent rows are handled by rank-revealing local linear algebra.
   * Exhausting the budget reports failure, with no global QP or AL fallback.
   */
  Result refine(const gtsam::GaussianFactorGraph& cost,
                const gtsam::VectorValues& initial,
                const IndexSet& blockingIndices,
                const gtsam::NonlinearOptimizerParams& params,
                size_t maxIterations) const;

 private:
  struct Block {
    gtsam::Key key;
    gtsam::Matrix rows;
    gtsam::Matrix whitenedRows;
  };
  std::vector<Block> blocks_;
};
}  // namespace internal
}  // namespace gtdynamics
