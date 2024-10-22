/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTwistAccelFactor.cpp
 * @brief Test twistAccel factor.
 * @author Frank Dellaert and Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/TwistAccelFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>
#include <iostream>

#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);

gtsam::Key qKey = JointAngleKey(1), qVelKey = JointVelKey(1),
           qAccelKey = JointAccelKey(1), twistKey = TwistKey(2),
           twistAccel_p_key = TwistAccelKey(1),
           twistAccel_c_key = TwistAccelKey(2);
}  // namespace example

// Test twistAccel factor for stationary case
TEST(TwistAccelFactor, error) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint_and_links = make_joint(cMp, screw_axis);
  auto joint = joint_and_links.first;

  // create factor
  auto factor = TwistAccelFactor(example::cost_model, joint, 0);
  double q = M_PI / 4, qVel = 10, qAccel = 10;
  gtsam::Vector twist, twistAccel_p, twistAccel_c;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twistAccel_p = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twistAccel_c =
      (gtsam::Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::qAccelKey, qAccel);
  values.insert(example::twistKey, twist);
  values.insert(example::twistAccel_p_key, twistAccel_p);
  values.insert(example::twistAccel_c_key, twistAccel_c);
  actual_errors = factor->unwhitenedError(values);
  expected_errors << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, diffDelta, 1e-3);
}

// Test twistAccel factor for stationary case
TEST(TwistAccelFactor, error_1) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis = (gtsam::Vector(6) << 0, 0, 1, 0, 1, 0).finished();

  auto joint_and_links = make_joint(cMp, screw_axis);
  auto joint = joint_and_links.first;

  auto factor = TwistAccelFactor(example::cost_model, joint, 0);
  double q = 0, qVel = 0, qAccel = -9.8;
  gtsam::Vector6 twist, twistAccel_p, twistAccel_c;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twistAccel_p = (gtsam::Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  twistAccel_c = (gtsam::Vector(6) << 0, 0, -9.8, 0, 0, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::qAccelKey, qAccel);
  values.insert(example::twistKey, twist);
  values.insert(example::twistAccel_p_key, twistAccel_p);
  values.insert(example::twistAccel_c_key, twistAccel_c);
  actual_errors = factor->unwhitenedError(values);
  expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
