/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file ProxQpSolver.h
 * @brief Internal bridge from a B-reduced GTSAM QP to sparse ProxQP.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Ordering.h>

#include <optional>
#include <string>
#include <vector>

namespace gtdynamics::internal {

/// Internal owning result; iterations are ProxQP iterations, not
/// factorizations.
struct ProxQpResult {
  gtsam::Values values;
  std::vector<bool> activeRows;
  size_t iterations = 0;
  bool converged = false;
  std::string message;
};

/** Solve a reduced vector QP with no explicit equalities. Pack costs and rows
 * sparsely, retaining original inequality block/row order and constraint signs.
 * The ordering controls scalar packing; ProxQP selects its own sparse ordering.
 * No permanent objective penalty is added. Require solved status and original-
 * factor KKT checks. Configuration/input errors throw; numerical failures
 * return an unconverged result. ProxSuite types remain private to the
 * implementation.
 */
ProxQpResult solveProxQp(const gtsam::QpProblem& problem,
                         const gtsam::Values& prototype,
                         const std::optional<gtsam::Ordering>& ordering,
                         size_t maxIterations);

}  // namespace gtdynamics::internal
