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

  auto params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  PenaltyOptimizer optimizer(problem, init_values, params);
  Values results = optimizer.optimize();

  /// Check the result is correct within tolerance.
  double tol = 1e-4;
  EXPECT(assert_equal(optimal_values, results, tol));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
