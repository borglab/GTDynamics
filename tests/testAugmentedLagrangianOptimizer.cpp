/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testAugmentedLagrangianOptimizr.cpp
 * @brief Test augmented Lagrangian method optimzier for equality constrained optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>

#include "constrainedExample.h"
#include "gtdynamics/optimizer/AugmentedLagrangianOptimizer.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;

TEST(AugmentedLagrangianOptimizer, ConstrainedExample) {
  using namespace constrained_example;
  Values values;
  values.insert(x1_key, -0.2);
  values.insert(x2_key, -0.2);

  NonlinearFactorGraph graph;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  auto constrained_noise = gtsam::noiseModel::Constrained::All(1, 1.0);

  std::cout << "cosntrained_noise\n";
  constrained_noise->print("");

  graph.add(ExpressionFactor<double>(cost_noise, 0., cost1_expr));
  graph.add(ExpressionFactor<double>(cost_noise, 0., cost2_expr));
  graph.add(
      ExpressionFactor<double>(constrained_noise, 0., constraint1_expr));

  gtdynamics::AugmentedLagrangianOptimizer optimizer;
  Values results = optimizer.optimize(graph, values);

  Values gt_results;
  gt_results.insert(x1_key, 0.0);
  gt_results.insert(x2_key, 0.0);
  double tol = 1e-3;
  EXPECT(assert_equal(gt_results, results, tol));

  // results.print();
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
