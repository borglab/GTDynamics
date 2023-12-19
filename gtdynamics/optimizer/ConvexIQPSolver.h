/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IQPSolver.h
 * @brief Solver of a convex linear inequality-constrained quadratic programming
 * problem.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

using gtdynamics::LinearInequalityConstraints;

namespace gtsam {

/** Solve a convex inequality constrained QP problem defined in
 * Nocedal Sec. 16.5 using the algorithm in Nocedal Alg. 16.3.
 * @param graph Cost for IQP probelm.
 * @param constraints Linear inequality constraints of IQP problem.
 * @param init_active_indices Initial indices of inequality constraints.
 * @param init_values Initial values for optimization.
 * @return [solution vector, active constriant indices, num solves, solve
 * successful]. */
std::tuple<VectorValues, IndexSet, size_t, bool>
SolveConvexIQP(const GaussianFactorGraph &graph,
               const LinearInequalityConstraints &constraints,
               const IndexSet &init_active_indices,
               const VectorValues &init_values,
               size_t max_iters = 100);

} // namespace gtsam
