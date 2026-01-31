/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testAugmentedLagrangianOptimizr.cpp
 * @brief Test augmented Lagrangian method optimzier for equality constrained
 * optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/constrained_optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>

#include "constrainedExample.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;

TEST(AugmentedLagrangianOptimizer, ConstrainedExample) {
  using namespace constrained_example;

  /// Create a constrained optimization problem with 2 cost factors and 1
  /// constraint.
  NonlinearFactorGraph graph;
  auto f1 = x1 + exp(-x2);
  auto f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));

  EqualityConstraints constraints;
  double tolerance = 1.0;
  auto g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  constraints.push_back(EqualityConstraint::shared_ptr(
      new DoubleExpressionEquality(g1, tolerance)));

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, -0.2);
  init_values.insert(x2_key, -0.2);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  gtdynamics::AugmentedLagrangianOptimizer optimizer;
  Values results = optimizer.optimize(graph, constraints, init_values);

  /// Check the result is correct within tolerance.
  Values gt_results;
  gt_results.insert(x1_key, 0.0);
  gt_results.insert(x2_key, 0.0);
  double tol = 1e-4;
  EXPECT(assert_equal(gt_results, results, tol));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
