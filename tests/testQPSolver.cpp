/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTangentCone.cpp
 * @brief test tangent cone.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/optimizer/QPSolver.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtsam;

/** Test solving 
  * argmin ||X-[1;1]||^2 
  * s.t.   [2,1] X = 1 */
TEST(SolveEQP, example_2D) {
  Matrix A_cost = Matrix22::Identity();
  Vector b_cost = Vector2(1, 1);
  Matrix A_constraint = (Matrix(1, 2) << 2, 1).finished();
  Vector b_constraint = Vector1(1);

  auto result = SolveEQP(A_cost, b_cost, A_constraint, b_constraint);
  Vector p = result.first;
  Vector lambda = result.second;
  Vector expected_p = Vector2(0.2, 0.6);
  EXPECT(assert_equal(expected_p, p));
  Vector expected_lambda = Vector1(-0.4);
  EXPECT(assert_equal(expected_lambda, lambda));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
