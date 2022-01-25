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

#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>

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
  gtdynamics::PenaltyMethodOptimizer penalty_optimizer;
  gtdynamics::ConstrainedOptResult penalty_info;
  Values penalty_results = penalty_optimizer.optimize(
      graph, constraints, init_values, &penalty_info);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  gtdynamics::AugmentedLagrangianOptimizer augl_optimizer;
  gtdynamics::ConstrainedOptResult augl_info;
  Values augl_results =
      augl_optimizer.optimize(graph, constraints, init_values, &augl_info);

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
  std::ofstream penalty_file;
  penalty_file.open("penalty_data.txt");
  for (size_t i = 0; i < penalty_info.num_iters.size(); i++) {
    penalty_file << penalty_info.num_iters[i] << " "
                 << penalty_info.mu_values[i] << " "
                 << evaluate_constraint(penalty_info.intermediate_values[i])
                 << " " << evaluate_cost(penalty_info.intermediate_values[i])
                 << "\n";
  }
  penalty_file.close();

  std::ofstream augl_file;
  augl_file.open("augl_data.txt");
  for (size_t i = 0; i < augl_info.num_iters.size(); i++) {
    augl_file << augl_info.num_iters[i] << " " << augl_info.mu_values[i] << " "
              << evaluate_constraint(augl_info.intermediate_values[i]) << " "
              << evaluate_cost(augl_info.intermediate_values[i]) << "\n";
  }
  augl_file.close();
  return 0;
}