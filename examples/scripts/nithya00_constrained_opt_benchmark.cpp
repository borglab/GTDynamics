/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  nithya_yetong00_constrainedopt_benchmark.cpp
 * @brief Benchmark penalty method optimizer and augmented lagrangian optimizer
 * on a toy example, and output intermediate results to file.
 * @author: Nithya Jayakumar
 * @author: Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/constrained_optimizer/PenaltyOptimizer.h>
#include <gtdynamics/constraints/EqualityConstraint.h>

#include <fstream>
#include <iostream>

#include "constrainedExample.h"

using namespace gtsam;
using namespace gtdynamics;

int main(int argc, char** argv) {
  using namespace constrained_example;

  /// Create a constrained optimization problem with 2 cost factors and 1
  /// constraint.
  NonlinearFactorGraph graph;
  gtsam::Double_ f1 = x1 + exp(-x2);
  gtsam::Double_ f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));

  EqualityConstraints constraints;
  double tolerance = 1.0;
  gtsam::Double_ g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  constraints.push_back(EqualityConstraint::shared_ptr(
      new DoubleExpressionEquality(g1, tolerance)));

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, -0.2);
  init_values.insert(x2_key, -0.2);

  /// Solve the constraint problem with Penalty Method optimizer.
  gtdynamics::PenaltyOptimizer penalty_optimizer;
  Values penalty_results =
      penalty_optimizer.optimize(graph, constraints, init_values);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  gtdynamics::AugmentedLagrangianOptimizer augl_optimizer;
  Values augl_results =
      augl_optimizer.optimize(graph, constraints, init_values);

  /// Function to evaluate constraint violation.
  auto evaluate_constraint = [&constraints](const gtsam::Values& values) {
    double violation = 0;
    for (auto& constraint : constraints) {
      violation += (*constraint)(values)(0);
    }
    return violation;
  };

  /// Function to evaluate cost.
  auto evaluate_cost = [&graph](const gtsam::Values& values) {
    double cost = graph.error(values);
    return cost;
  };

  /// Write results to files for plotting.
  std::cout << "Penalty result: cost=" << evaluate_cost(penalty_results)
            << " constraint=" << evaluate_constraint(penalty_results) << "\n";
  std::cout << "Augmented Lagrangian result: cost="
            << evaluate_cost(augl_results)
            << " constraint=" << evaluate_constraint(augl_results) << "\n";
  return 0;
}
