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
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "constrainedExample.h"
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "make_joint.h"

using namespace gtdynamics;
using namespace gtsam;
using constrained_example::x1_key, constrained_example::x2_key;

// Test methods of DoubleExpressionEquality.
TEST(EqualityConstraint, DoubleExpressionEquality) {
  // create constraint from double expression
  // g(x1, x2) = x1 + x1^3 + x2 + x2^2, from Vanderbergh slides
  double tolerance = 0.1;
  auto g = constrained_example::constraint1_expr;
  auto constraint = DoubleExpressionEquality(g, tolerance);

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, 0.0);
  values1.insert(x2_key, 0.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));

  // Check that violation evaluates as 0 at values1.
  EXPECT(assert_equal(Vector::Zero(1), constraint.evaluateViolation(values1)));
  EXPECT(assert_equal(Vector::Zero(1),
                      constraint.toleranceScaledViolation(values1)));

  // Check that values2 are indeed deemed infeasible.
  EXPECT(!constraint.feasible(values2));

  // Check constraint violation is indeed g(x) at values2.
  EXPECT(assert_equal(Vector::Constant(1, 4.0),
                      constraint.evaluateViolation(values2)));

  // Check scaled violation is indeed g(x)/tolerance at values2.
  EXPECT(assert_equal(Vector::Constant(1, 40.0),
                      constraint.toleranceScaledViolation(values2)));

  // Check dimension is 1 for scalar g.
  EXPECT(constraint.dim() == 1);

  // Generate factor representing the term in merit function.
  double mu = 4;
  Vector bias = Vector::Constant(1, 0.5);
  auto merit_factor = constraint.createFactor(mu, bias);

  // Check that noise model sigma == tolerance/sqrt(mu).
  auto expected_noise = noiseModel::Isotropic::Sigma(1, tolerance / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu * (g(x)+bias)^2/tolerance^2.
  double expected_error1 = 50;  // 0.5 * 4 * ||0 + 0.5||_(0.1^2)^2
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 4050;  // 0.5 * 4 * ||4 + 0.5||_(0.1^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Test methods of VectorExpressionEquality.
TEST(EqualityConstraint, VectorExpressionEquality) {
  // g(v1, v2) = v1 + v2, our own example.
  auto g = constrained_example::constraint_sum_vector2_expr;
  auto tolerance = Vector2(0.1, 0.5);
  auto constraint = VectorExpressionEquality<2>(g, tolerance);

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, Vector2(1, 1));
  values1.insert(x2_key, Vector2(-1, -1));
  values2.insert(x1_key, Vector2(1, 1));
  values2.insert(x2_key, Vector2(1, 1));

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));

  // Check that violation evaluates as 0 at values1.
  auto expected_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(
      assert_equal(expected_violation1, constraint.evaluateViolation(values1)));
  auto expected_scaled_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_scaled_violation1,
                      constraint.toleranceScaledViolation(values1)));

  // Check that values2 are indeed deemed infeasible.
  EXPECT(!constraint.feasible(values2));

  // Check constraint violation is indeed g(x) at values2.
  auto expected_violation2 = (Vector(2) << 2, 2).finished();
  EXPECT(
      assert_equal(expected_violation2, constraint.evaluateViolation(values2)));

  // Check scaled violation is indeed g(x)/tolerance at values2.
  auto expected_scaled_violation2 = (Vector(2) << 20, 4).finished();
  EXPECT(assert_equal(expected_scaled_violation2,
                      constraint.toleranceScaledViolation(values2)));

  // Check dim is the dimension of the vector.
  EXPECT(constraint.dim() == 2);

  // Generate factor representing the term in merit function.
  double mu = 4;
  Vector bias = (Vector(2) << 1, 0.5).finished();
  auto merit_factor = constraint.createFactor(mu, bias);

  // Check that noise model sigma == tolerance/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(tolerance / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu*||g(x)+bias)||^2_Diag(tolerance^2).
  double expected_error1 = 202;  // 0.5 * 4 * ||[1, 0.5]||_([0.1,0.5]^2)^2
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 1850;  // 0.5 * 4 * ||[3, 2.5]||_([0.1,0.5]^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Example of pose factor.
namespace pose_factor_example {
auto cost_model = gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key wTp_key = gtdynamics::internal::PoseKey(1),
           wTc_key = gtdynamics::internal::PoseKey(2),
           q_key = gtdynamics::internal::JointAngleKey(1);

Pose3 cMp = Pose3(Rot3(), Point3(-2, 0, 0));
Vector6 screw_axis = (Vector(6) << 0, 0, 1, 0, 1, 0).finished();
auto joint = make_joint(cMp, screw_axis);
auto factor = NoiseModelFactor::shared_ptr(
    new PoseFactor(wTp_key, wTc_key, q_key, cost_model, joint));
}  // namespace pose_factor_example

// Test methods of NoiseFactorEquality by creating a constrained PoseFactor.
TEST(EqualityConstraint, NoiseFactorEquality) {
  // create constraint from PoseFactor
  auto tolerance = (Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished();
  auto constraint = NoiseFactorEquality(pose_factor_example::factor, tolerance);

  // create 2 sets of values for testing
  Values values1, values2;
  InsertPose(&values1, 1, Pose3(Rot3(), Point3(1, 0, 0)));
  InsertPose(&values1, 2, Pose3(Rot3(), Point3(3, 0, 0)));
  InsertJointAngle(&values1, 1, 0.0);
  InsertPose(&values2, 1, Pose3(Rot3(), Point3(1, 0, 0)));
  InsertPose(&values2, 2, Pose3(Rot3(), Point3(5, 3, 3)));
  InsertJointAngle(&values2, 1, 0.0);

  // check method feasible
  EXPECT(constraint.feasible(values1));
  EXPECT(!constraint.feasible(values2));

  // check method evaluateViolation
  auto expected_violation1 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(
      assert_equal(expected_violation1, constraint.evaluateViolation(values1)));
  auto expected_violation2 = (Vector(6) << 0, 0, 0, -2, -3, -3).finished();
  EXPECT(
      assert_equal(expected_violation2, constraint.evaluateViolation(values2)));

  // check method toleranceScaledViolation
  auto expected_scaled_violation1 = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_scaled_violation1,
                      constraint.toleranceScaledViolation(values1)));
  auto expected_scaled_violation2 =
      (Vector(6) << 0, 0, 0, -4, -6, -6).finished();
  EXPECT(assert_equal(expected_scaled_violation2,
                      constraint.toleranceScaledViolation(values2)));

  // check method dim
  EXPECT(constraint.dim() == 6);

  // check method createFactor
  double mu = 4;
  Vector bias = (Vector(6) << 0, 0.1, 0, 0, 0.5, 0).finished();
  auto merit_factor = constraint.createFactor(mu, bias);

  // check noise model is correct
  auto sigmas = (Vector(6) << 0.05, 0.05, 0.05, 0.25, 0.25, 0.25).finished();
  auto expected_noise = noiseModel::Diagonal::Sigmas(sigmas);
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // check error is correct
  double expected_error1 = 4;  // 0.5 * 4 * (1 + 1)
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 156;  // 0.5 * 4 * (1 + 4*4 + 5*5 + 6*6)
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // check jacobian is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Test constraint container by adding different kinds of constraints.
TEST(EqualityConstraint, EqualityConstraints) {
  EqualityConstraints constraints;
  constraints.addDoubleExpressionEquality(constrained_example::constraint1_expr,
                                          0.1);
  constraints.addVectorExpressionEquality<2>(
      constrained_example::constraint_sum_vector2_expr, Vector2(0.1, 0.5));
  constraints.addNoiseFactorEquality(pose_factor_example::factor,
                                     Vector::Ones(6));
  constraints.addNoiseFactorEquality(pose_factor_example::factor);
  EXPECT(constraints.size() == 4);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
