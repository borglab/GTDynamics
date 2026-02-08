/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPenaltyOptimizr.cpp
 * @brief Test SQP optimzier for constrained optimization.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/constrained_optimizer/SQPOptimizer.h>

#include "constrainedExample.h"

using namespace gtsam;
using namespace gtdynamics;


/* ************************************************************************* */
// TEST(SQPOptimizer, EConstrainedExample) {
//   using namespace e_constrained_example;

//   /// Create initial values.
//   Values init_values;
//   init_values.insert(x1_key, -0.2);
//   init_values.insert(x2_key, -0.2);

//   /// Solve the constraint problem with Augmented Lagrangian optimizer.
//   SQPOptimizer optimizer;
//   Values results = optimizer.optimize(cost, constraints, init_values);

//   /// Check the result is correct within tolerance.
//   Values gt_results;
//   gt_results.insert(x1_key, 0.0);
//   gt_results.insert(x2_key, 0.0);
//   double tol = 1e-4;
//   EXPECT(assert_equal(gt_results, results, tol));

//   // // TODO: add check for lambda and mu values across iterations.
//   // for (int i=0; i<intermediate_result.mu_values.size(); i++) {
//   //   std::cout << i << "\t" << intermediate_result.mu_values[i] << "\t" <<
//   //   intermediate_result.num_iters[i] << "\n";
//   // }
// }

/* ************************************************************************* */
TEST(SQPOptimizer, IEConstrainedExample) {
  using namespace i_constrained_example;

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, 0.0);
  init_values.insert(x2_key, 0.0);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  SQPOptimizer optimizer;
  Values results =
      optimizer.optimize(cost, e_constraints, i_constraints, init_values);

  /// Check the result is correct within tolerance.
  Values gt_results;
  gt_results.insert(x1_key, sqrt(2) / 2);
  gt_results.insert(x2_key, sqrt(2) / 2);
  double tol = 1e-4;
  EXPECT(
      assert_equal(gt_results.atDouble(x1_key), results.atDouble(x1_key), tol));
  EXPECT(
      assert_equal(gt_results.atDouble(x2_key), results.atDouble(x2_key), tol));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
