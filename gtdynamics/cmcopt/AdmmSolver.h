/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file AdmmSolver.h
 * @brief Experimental ADMM backend for CMC tangent QPs.
 */
#pragma once

#include <gtdynamics/cmcopt/AdmmSplitting.h>
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

/** Parameters shared by manifold-variable and constraint-output splitting.
 * Constraints must be homogeneous and local to one reduced key. Both modes
 * reuse the global numerical factorization across ADMM iterations.
 */
struct AdmmParams {
  double rho = 1.0;
  /** Select the auxiliary variables. ConstraintOutputs uses orthant clipping
   * and requires a positive-definite damped objective; its original-factor
   * KKT checks run every 25 iterations and at exit.
   */
  AdmmSplitting splitting = AdmmSplitting::ManifoldVariables;
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

struct AdmmResult {
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
class AdmmWorkspace {
 public:
  /// Start without an analyzed factor graph.
  AdmmWorkspace();
  ~AdmmWorkspace();

  /** Solve a damped QP. Across calls, keep the factor order, types, scopes,
   * dimensions, elimination ordering and splitting mode fixed. Damping may
   * change.
   */
  AdmmResult solve(const gtsam::QpProblem& problem,
                   const gtsam::Values& prototype,
                   const std::optional<gtsam::Ordering>& ordering,
                   size_t maxIterations, const AdmmParams& params = {});

 private:
  /** Constraint-output split, sharing the fixed-structure multifrontal cache.
   */
  AdmmResult solveConstraints(const gtsam::QpProblem& problem,
                              const gtsam::Values& prototype,
                              const std::optional<gtsam::Ordering>& ordering,
                              size_t maxIterations, const AdmmParams& params);

  std::unique_ptr<gtsam::MultifrontalSolver> solver_;
};

/** Solve the *same* B-reduced QpProblem consumed by ProxQP.
 * Return active rows in original QpProblem order for CMC retraction.
 * Only a fully converged, original-factor KKT-checked result is accepted.
 * No fallback or implicit relaxation of the inequalities.
 */
AdmmResult solveAdmm(const gtsam::QpProblem& problem,
                     const gtsam::Values& prototype,
                     const std::optional<gtsam::Ordering>& ordering,
                     size_t maxIterations, const AdmmParams& params = {});

}  // namespace gtdynamics::internal
