/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testCollocationFactors.cpp
 * @brief Test collocation on link poses and twists.
 * @Author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "gtdynamics/factors/CollocationFactors.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Symbol pose_i_key('p', 1), pose_j_key('p', 2), twist_i_key('v', 1),
              twist_j_key('v', 2), accel_i_key('a', 1), accel_j_key('a', 2),
              dt_key('t', 0);
}  // namespace example

TEST(PoseTwistFunctor, error) {
  // create functor
  gtsam::Pose3 pose_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1));
  gtsam::Vector6 twist;
  twist << 1, 0, 0, 0, 0, 1;
  double dt = M_PI_2;
  gtsam::Vector6 twistdt = twist * dt;
  gtsam::Pose3 pose_j =
      gtsam::Pose3(gtsam::Rot3::Rx(M_PI_2), gtsam::Point3(0, -1, 2));
  //   gtsam::Pose3 pose_j = pose_i * gtsam::Pose3::Expmap(twistdt);

  gtdynamics::PoseTwistFunctor predictPose;
  EXPECT(assert_equal(pose_j, predictPose(pose_i, twistdt), 1e-6));

  // Create factor
  gtdynamics::EulerPoseColloFactor factor(
      example::pose_i_key, example::pose_j_key, example::twist_i_key,
      example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose_i, pose_j, twist, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::twist_i_key, twist);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, EulerPose) {
  // create functor
  gtsam::Pose3 pose_i = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.7, -0.5, 2),
                                     gtsam::Point3(0.4, -0.3, 0.9));
  gtsam::Vector6 twist;
  twist << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  double dt = M_PI_2;
  gtsam::Vector6 twistdt = twist * dt;
  gtsam::Pose3 pose_j = pose_i * gtsam::Pose3::Expmap(twistdt);

  gtdynamics::PoseTwistFunctor predictPose;
  EXPECT(assert_equal(pose_j, predictPose(pose_i, twistdt), 1e-6));

  // Create factor
  gtdynamics::EulerPoseColloFactor factor(
      example::pose_i_key, example::pose_j_key, example::twist_i_key,
      example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose_i, pose_j, twist, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::twist_i_key, twist);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, TrapezoidalPose) {
  // create functor
  gtsam::Pose3 pose_i = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.7, -0.5, 2),
                                     gtsam::Point3(0.4, -0.3, 0.9));
  gtsam::Vector6 twist_i, twist_j;
  twist_i << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_j << 0.6, 0.2, -0.1, 0.4, -0.8, -0.9;
  double dt = M_PI_2;
  gtsam::Vector6 twistdt = 0.5 * dt * (twist_i + twist_j);
  gtsam::Pose3 pose_j = pose_i * gtsam::Pose3::Expmap(twistdt);

  // Create factor
  gtdynamics::TrapezoidalPoseColloFactor factor(
      example::pose_i_key, example::pose_j_key, example::twist_i_key,
      example::twist_j_key, example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors =
      factor.evaluateError(pose_i, pose_j, twist_i, twist_j, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::twist_i_key, twist_i);
  values.insert(example::twist_j_key, twist_j);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, EulerTwist) {
  // create functor
  gtsam::Vector6 twist_i, twist_j, accel_i;
  twist_i << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_j << 0.3, -0.3, 0.1, 0.4, 0.2, 0.1;
  accel_i << 2, -9, -1, 5, -7, -9;
  double dt = 0.1;

  // Create factor
  gtdynamics::EulerTwistColloFactor factor(
      example::twist_i_key, example::twist_j_key, example::accel_i_key,
      example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(twist_i, twist_j, accel_i, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_i_key, twist_i);
  values.insert(example::twist_j_key, twist_j);
  values.insert(example::accel_i_key, accel_i);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, TrapezoidalTwist) {
  // create functor
  gtsam::Vector6 twist_i, twist_j, accel_i, accel_j;
  twist_i << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_j << 0.3, -0.3, 0.1, 0.4, 0.2, 0.1;
  accel_i << 1, -5, -2, 4, -6, -9;
  accel_j << 3, -13, 0, 6, -8, -9;
  double dt = 0.1;

  // Create factor
  gtdynamics::TrapezoidalTwistColloFactor factor(
      example::twist_i_key, example::twist_j_key, example::accel_i_key,
      example::accel_j_key, example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors =
      factor.evaluateError(twist_i, twist_j, accel_i, accel_j, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_i_key, twist_i);
  values.insert(example::twist_j_key, twist_j);
  values.insert(example::accel_i_key, accel_i);
  values.insert(example::accel_j_key, accel_j);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
