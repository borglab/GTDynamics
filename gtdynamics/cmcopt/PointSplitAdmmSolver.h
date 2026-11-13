/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file PointSplitAdmmSolver.h
 * @brief Experimental variable-splitting ADMM backend for CMC tangent QPs.
 */
#pragma once

#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Ordering.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gtsam {
class MultifrontalSolver;
}

namespace gtdynamics::internal {

/** One copy z_i for each reduced key, with z_i in its local tangent cone.
 * All local constraints must be homogeneous and involve only that key.
 * The global step factors H + rho*M once; local projections may run in
 * parallel.
 */
struct PointSplitAdmmParams {
  double rho = 1.0;
  /** Split inequality outputs instead of point coordinates. Projection becomes
   * clipping to the negative orthant. The damped objective must be positive
   * definite. Original-factor KKT checks run every 25 iterations and at exit.
   */
  bool useConstraintSplitting = false;
  /** Use M = diag(max(diag(H), 1e-12)) for the damped QP, otherwise M = I.
   * The same fixed metric is used in the penalty and local cone projections.
   * With constraint splitting, instead change coordinates x = D y using
   * D = diag(max(diag(H), 1e-12))^{-1/2} and normalize rows of G D.
   * The physical objective, constraint rows and original-factor KKT checks
   * are unchanged. The metric is recomputed for each QP/damping trial.
   */
  bool useDiagonalMetric = false;
  double absoluteTolerance = 1e-9;
  double relativeTolerance = 1e-9;
  size_t maxProjectionSweeps = 2000;
  size_t projectionThreads = 1;
};

struct PointSplitAdmmResult {
  gtsam::Values values;
  std::vector<bool> activeRows;
  size_t iterations = 0;
  bool converged = false;
  std::string message;
  /// Consensus infinity norm (in row-scaled units for constraint splitting).
  double primalResidual = 0;
  /// Point mode: dual step residual. Constraint mode: last physical
  /// stationarity.
  double dualResidual = 0;
  /// NaN until the original-factor KKT stationarity check has been evaluated.
  double stationarityResidual = std::numeric_limits<double>::quiet_NaN();
  double factorizationSeconds = 0;
  double projectionSeconds = 0;
};

/** Reuse multifrontal symbolic analysis for trials at one LM linearization. */
class PointSplitAdmmWorkspace {
 public:
  /// Start without an analyzed factor graph.
  PointSplitAdmmWorkspace();
  ~PointSplitAdmmWorkspace();

  /** Solve a damped QP. Across calls, keep the factor order, types, scopes,
   * dimensions, elimination ordering and splitting mode fixed. Damping may
   * change.
   */
  PointSplitAdmmResult solve(const gtsam::QpProblem& problem,
                             const gtsam::Values& prototype,
                             const std::optional<gtsam::Ordering>& ordering,
                             size_t maxIterations,
                             const PointSplitAdmmParams& params = {});

 private:
  /** Constraint-output split, sharing the fixed-structure multifrontal cache.
   */
  PointSplitAdmmResult solveConstraints(
      const gtsam::QpProblem& problem, const gtsam::Values& prototype,
      const std::optional<gtsam::Ordering>& ordering, size_t maxIterations,
      const PointSplitAdmmParams& params);

  std::unique_ptr<gtsam::MultifrontalSolver> solver_;
};

/** Solve the *same* B-reduced QpProblem consumed by ProxQP.
 * Return active rows in original QpProblem order for CMC retraction.
 * Only a fully converged, original-factor KKT-checked result is accepted.
 * No fallback or implicit relaxation of the inequalities.
 */
PointSplitAdmmResult solvePointSplitAdmm(
    const gtsam::QpProblem& problem, const gtsam::Values& prototype,
    const std::optional<gtsam::Ordering>& ordering, size_t maxIterations,
    const PointSplitAdmmParams& params = {});

}  // namespace gtdynamics::internal
