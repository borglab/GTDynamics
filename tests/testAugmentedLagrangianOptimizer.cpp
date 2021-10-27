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

#include "constrainedExample.h"
#include "gtdynamics/optimizer/AugmentedLagrangianOptimizer.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"

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
  EqualityConstraints constraints;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., cost1_expr));
  graph.add(ExpressionFactor<double>(cost_noise, 0., cost2_expr));
  double tolerance = 1.0;
  constraints.push_back(EqualityConstraint::shared_ptr(
      new DoubleExpressionEquality(constraint1_expr, tolerance)));

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
