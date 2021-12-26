/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyMethodOptimizer.cpp
 * @brief Penalty method optimization routines.
 * @author: Yetong Zhang
 */

#include "gtdynamics/optimizer/PenaltyMethodOptimizer.h"

namespace gtdynamics {

gtsam::Values PenaltyMethodOptimizer::optimize(
    const gtsam::NonlinearFactorGraph& graph,
    const EqualityConstraints& constraints, const gtsam::Values& initial_values,
    ConstrainedOptResult* intermediate_result) const {
  gtsam::Values values = initial_values;
  double mu = p_.initial_mu;

  // Solve the constrained optimization problem by solving a sequence of
  // unconstrained optimization problems.
  for (int i = 0; i < p_.num_iterations; i++) {
    gtsam::NonlinearFactorGraph merit_graph = graph;

    // Create factors corresponding to penalty terms of constraints.
    for (auto& constraint : constraints) {
      merit_graph.add(constraint->createFactor(mu));
    }

    // Run optimization.
    gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                 p_.lm_parameters);
    auto result = optimizer.optimize();

    // Save results and update parameters.
    values = result;
    mu *= p_.mu_increase_rate;

    /// Store intermediate results.
    if (intermediate_result != nullptr) {
      intermediate_result->intermediate_values.push_back(values);
      intermediate_result->num_iters.push_back(optimizer.getInnerIterations());
      intermediate_result->mu_values.push_back(mu);
    }
  }
  return values;
}

}  // namespace gtdynamics
