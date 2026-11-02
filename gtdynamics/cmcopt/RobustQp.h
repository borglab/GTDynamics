/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/constrained/ActiveSetSolver.h>

namespace gtdynamics {
namespace internal {
/** Solve a CMC quadratic subproblem without discarding dependent active faces.
 * Dense active-set Schur is the normal fast path. Sparse active-set elimination
 * and finally native GTSAM AL handle degenerate faces of wrench cones. All
 * backends solve the same QP, with no added objective regularization. The AL
 * fallback reports rows having positive dual weight and a tight primal value.
 */
gtsam::ActiveSetSolverState robustQp(
    const gtsam::QpProblem& problem, const gtsam::Values& initial,
    const gtsam::ActiveSetSolver::State& workingSet, size_t maxIterations);
}  // namespace internal
}  // namespace gtdynamics
