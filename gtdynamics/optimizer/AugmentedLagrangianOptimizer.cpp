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

gtsam::Values AugmentedLagrangianOptimizer::optimize(
    const gtsam::NonlinearFactorGraph& graph,
    const EqualityConstraints& constraints,
    const gtsam::Values& initial_values) const {
  gtsam::Values values = initial_values;

  double mu = 1.0;               // penalty parameter
  std::vector<gtsam::Vector> z;  // Lagrangian multiplier
  for (const auto& constraint : constraints) {
    z.push_back(gtsam::Vector::Zero(constraint->dim()));
  }

  for (int i = 0; i < p_->num_iterations; i++) {
    // std::cout << "------------------ " << i << " ------------------\n";

    // construct merit function
    gtsam::NonlinearFactorGraph merit_graph = graph;
    for (size_t constraint_index = 0; constraint_index < constraints.size();
         constraint_index++) {
      auto constraint = constraints.at(constraint_index);
      gtsam::Vector bias = z[constraint_index] / mu;
      merit_graph.add(constraint->createFactor(mu, bias));
    }

    // run LM optimization
    gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                 p_->lm_parameters);
    auto result = optimizer.optimize();

    // update parameters
    double previous_error = 0;
    double current_error = 0;
    for (size_t constraint_index = 0; constraint_index < constraints.size();
         constraint_index++) {
      auto constraint = constraints.at(constraint_index);

      // update multipliers
      auto violation = (*constraint)(result);
      z[constraint_index] += mu * violation;

      // update penalty parameter
      previous_error += pow(constraint->toleranceScaledViolation(values).norm(), 2);
      current_error += pow(constraint->toleranceScaledViolation(result).norm(), 2);
    }
    if (sqrt(current_error) >= 0.25 * sqrt(previous_error)) {
      mu *= 2;
    }

    // update values
    values = result;
  }
  return values;
}

}  // namespace gtdynamics