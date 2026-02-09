/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPenaltyMethodOptimizr.cpp
 * @brief Test penalty method optimzier for equality constrained optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/PenaltyOptimizer.h>

#include "gtsamConstrainedExample.h"

using namespace gtsam;

TEST(PenaltyOptimizer, ConstrainedExample) {
  using namespace constrained_example1;

  using namespace constrained_example;

  /// Create a constrained optimization problem with 2 cost factors and 1
  /// constraint.
  NonlinearFactorGraph graph;
  auto f1 = x1 + exp(-x2);
  auto f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));

  Vector sigmas = Vector1(0.1);
  auto g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  graph.push_back(gtsam::NonlinearEqualityConstraint::shared_ptr(
      new gtsam::ExpressionEqualityConstraint<double>(g1, 0.0, sigmas)));

  /// Create initial values.
  Values initialValues;
  initialValues.insert(x1_key, -0.2);
  initialValues.insert(x2_key, -0.2);

  /// Solve the constraint problem with Penalty method optimizer.
  gtsam::PenaltyOptimizer optimizer(graph, initialValues);
  Values results = optimizer.optimize();

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
