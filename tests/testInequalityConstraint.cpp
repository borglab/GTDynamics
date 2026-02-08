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
#include <gtdynamics/constraints/LinearInequalityConstraint.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
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
  auto constraint = ScalarExpressionInequalityConstraint::GeqZero(g, tolerance);

  // Create 2 sets of values for testing.
  Values values1, values2, values3;
  values1.insert(x1_key, -1.0);
  values1.insert(x2_key, 1.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);
  values3.insert(x1_key, -2.0);
  values3.insert(x2_key, 1.0);

  // Check that values1 are feasible.
  EXPECT(constraint->feasible(values1));
  EXPECT(constraint->feasible(values2));
  EXPECT(!constraint->feasible(values3));

  // Check that expression evaluates correctly.
  EXPECT(assert_equal(Vector1(0.0), constraint->unwhitenedExpr(values1)));
  EXPECT(assert_equal(Vector1(-4.0), constraint->unwhitenedExpr(values2)));
  EXPECT(assert_equal(Vector1(8.0), constraint->unwhitenedExpr(values3)));

  EXPECT(assert_equal(0.0, constraint->violation(values1)));
  EXPECT(assert_equal(0.0, constraint->violation(values2)));
  EXPECT(assert_equal(80.0, constraint->violation(values3)));

  // Check dimension is 1 for scalar g.
  EXPECT(constraint->dim() == 1);
  EXPECT(assert_equal(Vector1(tolerance), constraint->sigmas()));

  // Check keys include x1, x2.
  EXPECT(constraint->keys().size() == 2);
  EXPECT(x1_key == *constraint->keys().begin());
  EXPECT(x2_key == *constraint->keys().rbegin());

  // Check create equality constraint
  auto equality_constraint = constraint->createEqualityConstraint();
  EXPECT(assert_equal(40.0, equality_constraint->violation(values2)));
}

TEST(InequalityConstraint, TwinDoubleExpressionInequality) {
  Double_ x1_expr(x1_key);
  Double_ x2_expr(x2_key);
  double tolerance = 0.1;

  auto constraint1 =
      ScalarExpressionInequalityConstraint::GeqZero(x1_expr, tolerance);
  auto constraint2 =
      ScalarExpressionInequalityConstraint::GeqZero(x2_expr, tolerance);

  NonlinearInequalityConstraints constraints;
  constraints.push_back(constraint1);
  constraints.push_back(constraint2);
  EXPECT_LONGS_EQUAL(2, constraints.dim());

  Values values1, values2, values3;
  values1.insert(x1_key, 2.0);
  values1.insert(x2_key, 1.0);
  values2.insert(x1_key, -2.0);
  values2.insert(x2_key, -1.0);
  values3.insert(x1_key, 0.0);
  values3.insert(x2_key, 0.0);

  EXPECT(constraint1->feasible(values1));
  EXPECT(constraint2->feasible(values1));
  EXPECT(!constraint1->feasible(values2));
  EXPECT(!constraint2->feasible(values2));

  EXPECT(assert_equal(Vector2(0, 0), constraints.violationVector(values1)));
  EXPECT(constraints.violationVector(values2).norm() > 0);
  EXPECT(assert_equal(Vector2(0, 0), constraints.violationVector(values3)));

  EXPECT(constraints.keys().size() == 2);

  auto penalty_graph = constraints.penaltyGraph(1.0);
  EXPECT(assert_equal(0.0, penalty_graph.error(values1)));
  EXPECT(assert_equal(250.0, penalty_graph.error(values2)));
  EXPECT(assert_equal(0.0, penalty_graph.error(values3)));

  auto l2_factor = constraint1->penaltyFactorEquality(1.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*l2_factor, values1, 1e-7, 1e-5);
}

TEST(LinearInequalityConstraint, JacobianLinearInequalityConstraint) {
  MultiJacobian jac;
  jac.addJacobian(x1_key, (Matrix(2, 2) << 2, -2, 1, 3).finished());
  jac.addJacobian(x2_key, (Matrix(2, 1) << 1, 2).finished());
  Vector b = Vector::Zero(2);
  SharedDiagonal model = noiseModel::Unit::Create(2);
  auto factor = std::make_shared<JacobianFactor>(jac, b, model);
  JacobianLinearInequalityConstraint constraint(factor);

  EXPECT_LONGS_EQUAL(2, constraint.dim());

  VectorValues values1;
  values1.insert(x1_key, Vector2(0, 0));
  values1.insert(x2_key, Vector1(0));
  VectorValues values2;
  values2.insert(x1_key, Vector2(0, 1));
  values2.insert(x2_key, Vector1(0));
  EXPECT(constraint.feasible(values1));
  EXPECT(!constraint.feasible(values2));

  auto constrained_factor = constraint.createConstrainedFactor();
  EXPECT(constrained_factor->get_model()->isConstrained());
  EXPECT(jac.equals(constraint.jacobian()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
