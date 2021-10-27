/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  AugmentedLagrangianOptimizer.cpp
 * @brief Augmented Lagrangian optimization routines.
 * @author: Yetong Zhang, Frank Dellaert
 */

#include "gtdynamics/optimizer/AugmentedLagrangianOptimizer.h"

namespace gtdynamics {

/** Update penalty parameter and Lagrangian multipliers from unconstrained
 * optimization result. */
void update_parameters(const EqualityConstraints& constraints,
                       const gtsam::Values& previous_values,
                       const gtsam::Values& current_values, double& mu,
                       std::vector<gtsam::Vector>& z) {
  double previous_error = 0;
  double current_error = 0;
  for (size_t constraint_index = 0; constraint_index < constraints.size();
       constraint_index++) {
    auto constraint = constraints.at(constraint_index);

    // Update Lagrangian multipliers.
    auto violation = (*constraint)(current_values);
    z[constraint_index] += mu * violation;

    // Sum errors for updating penalty parameter.
    previous_error +=
        pow(constraint->toleranceScaledViolation(previous_values).norm(), 2);
    current_error +=
        pow(constraint->toleranceScaledViolation(current_values).norm(), 2);
  }

  // Update penalty parameter.
  if (sqrt(current_error) >= 0.25 * sqrt(previous_error)) {
    mu *= 2;
  }
}

gtsam::Values AugmentedLagrangianOptimizer::optimize(
    const gtsam::NonlinearFactorGraph& graph,
    const EqualityConstraints& constraints,
    const gtsam::Values& initial_values) const {
  gtsam::Values values = initial_values;

  // Set initial values for penalty parameter and Lagrangian multipliers.
  double mu = 1.0;               // penalty parameter
  std::vector<gtsam::Vector> z;  // Lagrangian multiplier
  for (const auto& constraint : constraints) {
    z.push_back(gtsam::Vector::Zero(constraint->dim()));
  }

  for (int i = 0; i < p_->num_iterations; i++) {
    // Construct merit function.
    gtsam::NonlinearFactorGraph merit_graph = graph;

    // Create factors corresponding to penalty terms of constraints.
    for (size_t constraint_index = 0; constraint_index < constraints.size();
         constraint_index++) {
      auto constraint = constraints.at(constraint_index);
      gtsam::Vector bias = z[constraint_index] / mu;
      merit_graph.add(constraint->createFactor(mu, bias));
    }

    // Run LM optimization.
    gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                 p_->lm_parameters);
    auto result = optimizer.optimize();

    // Update parameters.
    update_parameters(constraints, values, result, mu, z);

    // Update values.
    values = result;
  }
  return values;
}

}  // namespace gtdynamics