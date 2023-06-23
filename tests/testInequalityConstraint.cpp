/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testEqualityConstraintFactor.cpp
 * @brief Test Equality Constraint Factor.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include "constrainedExample.h"
#include "make_joint.h"

using namespace gtdynamics;
using namespace gtsam;
using constrained_example::pow;
using constrained_example::x1, constrained_example::x2;
using constrained_example::x1_key, constrained_example::x2_key;

// Test methods of DoubleExpressionEquality.
TEST(InequalityConstraint, DoubleExpressionInequality) {
  // create constraint from double expression
  // g(x1, x2) = x1 + x1^3 + x2 + x2^2, from Vanderbergh slides
  double tolerance = 0.1;
  auto g = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  auto constraint = DoubleExpressionInequality(g, tolerance);

  // Create 2 sets of values for testing.
  Values values1, values2, values3;
  values1.insert(x1_key, -1.0);
  values1.insert(x2_key, 1.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);
  values3.insert(x1_key, -2.0);
  values3.insert(x2_key, 1.0);

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));
  EXPECT(constraint.feasible(values2));
  EXPECT(!constraint.feasible(values3));

  // Check that violation evaluates as 0 at values1.
  EXPECT(assert_equal(0.0, constraint(values1)));
  EXPECT(assert_equal(4.0, constraint(values2)));
  EXPECT(assert_equal(-8.0, constraint(values3)));

  EXPECT(assert_equal(0.0, constraint.toleranceScaledViolation(values1)));
  EXPECT(assert_equal(0.0, constraint.toleranceScaledViolation(values2)));
  EXPECT(assert_equal(80.0, constraint.toleranceScaledViolation(values3)));

  // Check dimension is 1 for scalar g.
  EXPECT(constraint.dim() == 1);

  // Check keys include x1, x2.
  EXPECT(constraint.keys().size() == 2);
  EXPECT(x1_key == *constraint.keys().begin());
  EXPECT(x2_key == *constraint.keys().rbegin());

  // Check create equality constraint
  auto equality_constraint = constraint.createEqualityConstraint();
  EXPECT(assert_equal(Vector::Constant(1, 40.0),
                      equality_constraint->toleranceScaledViolation(values2)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
