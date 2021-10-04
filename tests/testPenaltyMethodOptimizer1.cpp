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

#include "constrainedExample.h"
#include "gtdynamics/optimizer/EqualityConstraintFactor.h"
#include "gtdynamics/optimizer/PenaltyMethodOptimizer.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;

TEST(PenaltyMethodOptimizer, ConstrainedExample) {
  using namespace constrained_example;
  Values values;
  values.insert(x1_key, -0.2);
  values.insert(x2_key, -0.2);

  NonlinearFactorGraph graph;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  auto constrained_noise = gtsam::noiseModel::Constrained::All(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., cost1_expr));
  graph.add(ExpressionFactor<double>(cost_noise, 0., cost2_expr));
  graph.add(EqualityConstraintFactor<double>(constraint1_expr, 0.0));

  PenaltyMethodOptimizer optimizer;
  Values results = optimizer.optimize1(graph, values);

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
