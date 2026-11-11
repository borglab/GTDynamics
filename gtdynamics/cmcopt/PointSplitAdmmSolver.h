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
#include <optional>
#include <string>
#include <vector>

namespace gtdynamics::internal {

/** One copy z_i for each reduced key, with z_i in its local tangent cone.
 * All local constraints must be homogeneous and involve only that key.
 * The global step factors H + rho*I once; local projections may run in parallel.
 */
struct PointSplitAdmmParams {
  double rho = 1.0;
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
  double primalResidual = 0;
  double dualResidual = 0;
  double stationarityResidual = 0;
  double factorizationSeconds = 0;
  double projectionSeconds = 0;
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
