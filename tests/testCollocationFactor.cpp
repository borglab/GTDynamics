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

using namespace gtdynamics;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Vector6, gtsam::Rot3;

namespace example {
// noise model
auto cost_model = gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Symbol pose_p_key('p', 1), pose_c_key('p', 2), twist_p_key('v', 1),
              twist_c_key('v', 2), accel_p_key('a', 1), accel_c_key('a', 2),
              dt_key('t', 0);
}  // namespace example

TEST(PoseTwistFunctor, error) {
  // create functor
  Pose3 pose_p = Pose3(Rot3(), gtsam::Point3(0, 0, 1));
  Vector6 twist;
  twist << 1, 0, 0, 0, 0, 1;
  double dt = M_PI_2;
  Vector6 twistdt = twist * dt;
  Pose3 pose_c(Rot3::Rx(M_PI_2), gtsam::Point3(0, -1, 2));

  EXPECT(assert_equal(pose_c, predictPose(pose_p, twistdt), 1e-6));

  // Create factor
  EulerPoseColloFactor factor(example::pose_p_key, example::pose_c_key,
                              example::twist_p_key, example::dt_key,
                              example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose_p, pose_c, twist, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::twist_p_key, twist);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, EulerPose) {
  // create functor
  Pose3 pose_p(Rot3::RzRyRx(0.7, -0.5, 2), gtsam::Point3(0.4, -0.3, 0.9));
  Vector6 twist;
  twist << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  double dt = M_PI_2;
  Vector6 twistdt = twist * dt;
  Pose3 pose_c = pose_p * Pose3::Expmap(twistdt);

  EXPECT(assert_equal(pose_c, predictPose(pose_p, twistdt), 1e-6));

  // Create factor
  EulerPoseColloFactor factor(example::pose_p_key, example::pose_c_key,
                              example::twist_p_key, example::dt_key,
                              example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose_p, pose_c, twist, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::twist_p_key, twist);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, TrapezoidalPose) {
  // create functor
  Pose3 pose_p(Rot3::RzRyRx(0.7, -0.5, 2), gtsam::Point3(0.4, -0.3, 0.9));
  Vector6 twist_p, twist_c;
  twist_p << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_c << 0.6, 0.2, -0.1, 0.4, -0.8, -0.9;
  double dt = M_PI_2;
  Vector6 twistdt = 0.5 * dt * (twist_p + twist_c);
  Pose3 pose_c = pose_p * Pose3::Expmap(twistdt);

  // Create factor
  TrapezoidalPoseColloFactor factor(example::pose_p_key, example::pose_c_key,
                                    example::twist_p_key, example::twist_c_key,
                                    example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors =
      factor.evaluateError(pose_p, pose_c, twist_p, twist_c, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, EulerTwist) {
  // create functor
  Vector6 twist_p, twist_c, accel_p;
  twist_p << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_c << 0.3, -0.3, 0.1, 0.4, 0.2, 0.1;
  accel_p << 2, -9, -1, 5, -7, -9;
  double dt = 0.1;

  // Create factor
  EulerTwistColloFactor factor(example::twist_p_key, example::twist_c_key,
                               example::accel_p_key, example::dt_key,
                               example::cost_model);

  // call evaluateError
  auto actual_errors = factor.evaluateError(twist_p, twist_c, accel_p, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);
  values.insert(example::accel_p_key, accel_p);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(RandomData, TrapezoidalTwist) {
  // create functor
  Vector6 twist_p, twist_c, accel_p, accel_c;
  twist_p << 0.1, 0.6, 0.2, -0.1, 0.9, 1;
  twist_c << 0.3, -0.3, 0.1, 0.4, 0.2, 0.1;
  accel_p << 1, -5, -2, 4, -6, -9;
  accel_c << 3, -13, 0, 6, -8, -9;
  double dt = 0.1;

  // Create factor
  TrapezoidalTwistColloFactor factor(example::twist_p_key, example::twist_c_key,
                                     example::accel_p_key, example::accel_c_key,
                                     example::dt_key, example::cost_model);

  // call evaluateError
  auto actual_errors =
      factor.evaluateError(twist_p, twist_c, accel_p, accel_c, dt);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);
  values.insert(example::accel_p_key, accel_p);
  values.insert(example::accel_c_key, accel_c);
  values.insert(example::dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
