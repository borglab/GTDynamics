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
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
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
TEST(EqualityConstraint, DoubleExpressionEquality) {
  // create constraint from double expression
  // g(x1, x2) = x1 + x1^3 + x2 + x2^2, from Vanderbergh slides
  double tolerance = 0.1;
  auto g = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  ;
  gtsam::ExpressionEqualityConstraint<double> constraint(
      g, 0.0, gtsam::Vector1(tolerance));

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, 0.0);
  values1.insert(x2_key, 0.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);

  // Check unwhitened error evaluates as g(x) at values2.
  EXPECT(assert_equal(Vector::Constant(1, 4.0),
                      constraint.unwhitenedError(values2)));

  // Check dimension is 1 for scalar g.
  EXPECT(constraint.dim() == 1);

  // Check keys include x1, x2.
  EXPECT(constraint.keys().size() == 2);
  EXPECT(x1_key == *constraint.keys().begin());
  EXPECT(x2_key == *constraint.keys().rbegin());

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint.penaltyFactor(mu);

  // Check that noise model sigma == tolerance/sqrt(mu).
  auto expected_noise = noiseModel::Isotropic::Sigma(1, tolerance / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu * g(x)^2/tolerance^2.
  double expected_error1 = 0;  // g(x)=0 at values1
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 3200;  // 0.5 * 4 * (4/0.1)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Test methods of VectorExpressionEquality.
TEST(EqualityConstraint, VectorExpressionEquality) {
  // g(v1, v2) = v1 + v2, our own example.
  Vector2_ x1_vec_expr(x1_key);
  Vector2_ x2_vec_expr(x2_key);
  auto g = x1_vec_expr + x2_vec_expr;
  auto tolerance = Vector2(0.1, 0.5);
  gtsam::ExpressionEqualityConstraint<gtsam::Vector2> constraint(
      g, gtsam::Vector2::Zero(), tolerance);

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, Vector2(1, 1));
  values1.insert(x2_key, Vector2(-1, -1));
  values2.insert(x1_key, Vector2(1, 1));
  values2.insert(x2_key, Vector2(1, 1));

  // Check unwhitened error evaluates as g(x) at values2.
  auto expected_violation2 = (Vector(2) << 2, 2).finished();
  EXPECT(assert_equal(expected_violation2, constraint.unwhitenedError(values2)));

  // Check dim is the dimension of the vector.
  EXPECT(constraint.dim() == 2);

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint.penaltyFactor(mu);

  // Check that noise model sigma == tolerance/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(tolerance / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu*||g(x)||^2_Diag(tolerance^2).
  double expected_error1 = 0;  // g(x)=0 at values1
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 832;  // 0.5 * 4 * ||[2,2]||_([0.1,0.5]^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

TEST(EqualityConstraint, Container) {
  gtsam::NonlinearEqualityConstraints constraints;

  double tolerance1 = 0.1;
  auto g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  ;

  Vector2_ x1_vec_expr(x1_key);
  Vector2_ x2_vec_expr(x2_key);
  auto g2 = x1_vec_expr + x2_vec_expr;
  auto tolerance2 = Vector2(0.1, 0.5);

  constraints.emplace_shared<gtsam::ExpressionEqualityConstraint<double>>(
      g1, 0.0, gtsam::Vector1(tolerance1));
  constraints.emplace_shared<
      gtsam::ExpressionEqualityConstraint<gtsam::Vector2>>(
      g2, gtsam::Vector2::Zero(), tolerance2);

  EXPECT_LONGS_EQUAL(2, constraints.size());
}

// Test methods of FactorZeroErrorConstraint.
TEST(EqualityConstraint, FactorZeroErrorConstraint) {
  Key x1_key = 1;
  Key x2_key = 2;
  Vector tolerance = Vector2(0.5, 0.1);
  auto noise = noiseModel::Diagonal::Sigmas(tolerance);
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Vector2>>(x1_key, x2_key, Vector2(1, 1),
                                                 noise);
  auto constraints = gtsam::NonlinearEqualityConstraints::FromCostGraph(graph);
  auto constraint = constraints.at(0);

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, Vector2(1, 1));
  values1.insert(x2_key, Vector2(2, 2));
  values2.insert(x1_key, Vector2(0, 0));
  values2.insert(x2_key, Vector2(2, 3));

  // Check unwhitened error is factor error.
  auto expected_violation2 = (Vector(2) << 1, 2).finished();
  EXPECT(assert_equal(expected_violation2, constraint->unwhitenedError(values2)));

  // Check dim is the dimension of the vector.
  EXPECT(constraint->dim() == 2);

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint->penaltyFactor(mu);

  // Check that noise model sigma == tolerance/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(tolerance / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu*||g(x)||^2_Diag(tolerance^2).
  double expected_error1 = 0;
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 808; // 0.5 * 4 * ||[1, 2]||_([0.5,0.1]^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
