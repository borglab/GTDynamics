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
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>

#include "constrainedExample.h"

using namespace gtsam;


/* ************************************************************************* */
// NOTE(Frank): Commented out until CMCOPt is implemented.
// TEST(LagrangeDualFunction, IConstrainedExample) {
//   using namespace i_constrained_example;
//   {
//     double mu_e = 0.2;
//     double mu_i = 0.3;
//     std::vector<Vector> lambda_e;
//     std::vector<double> lambda_i{2.0};
//     double d = 1e-6;
//     NonlinearFactorGraph dual_graph =
//         AugmentedLagrangianOptimizer::LagrangeDualFunction(
//             cost, e_constraints, i_constraints, mu_e, mu_i, lambda_e, lambda_i,
//             d);

//     Values values;
//     values.insert(x1_key, -1.0);
//     values.insert(x2_key, -1.0);
//     double g = -1 / 0.2;
//     double expected_error1 = cost.error(values) - 2.0 * g +
//                              0.5 * mu_i * pow(g, 2) + 0.5 * d * pow(g, 2) +
//                              pow(2.0, 2) / (2 * d);
//     EXPECT_DOUBLES_EQUAL(expected_error1, dual_graph.error(values), 1e-9);
//   }
// }

/* ************************************************************************* */
// NOTE(Frank): Commented out until CMCOPt is implemented.
// TEST(AugmentedLagrangianOptimizer, IEConstrainedExample) {
//   using namespace i_constrained_example;

//   /// Create initial values.
//   Values init_values;
//   init_values.insert(x1_key, 0.0);
//   init_values.insert(x2_key, 0.0);

//   /// Solve the constraint problem with Augmented Lagrangian optimizer.
//   gtsam::AugmentedLagrangianOptimizer optimizer;
//   Values results =
//       optimizer.optimize(cost, e_constraints, i_constraints, init_values);

//   /// Check the result is correct within tolerance.
//   Values gt_results;
//   gt_results.insert(x1_key, sqrt(2) / 2);
//   gt_results.insert(x2_key, sqrt(2) / 2);
//   double tol = 1e-4;
//   EXPECT(
//       assert_equal(gt_results.atDouble(x1_key), results.atDouble(x1_key), tol));
//   EXPECT(
//       assert_equal(gt_results.atDouble(x2_key), results.atDouble(x2_key), tol));
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
