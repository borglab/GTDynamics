/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QPSolver.h
 * @brief Solver of a quadratic programming problem.
 * @author: Yetong Zhang
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#pragma once

namespace gtsam {

/** Solve an equality constrained QP problem defined by
 * Nocedal 16.39.
 * Returns resulting vector and Lagrangian multipliers. */
std::pair<Vector, Vector> SolveEQP(const Matrix &A_cost, const Vector &b_cost,
                                   const Matrix &A_constraint,
                                   const Vector &b_constraint);

} // namespace gtsam
