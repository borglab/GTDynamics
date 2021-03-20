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

#include "gtdynamics/factors/TwistAccelFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key qKey = gtsam::Symbol('q', 0), qVelKey = gtsam::Symbol('j', 0),
           qAccelKey = gtsam::Symbol('a', 0), twistKey = gtsam::Symbol('V', 0),
           twistAccel_p_key = gtsam::Symbol('T', 0),
           twistAccel_c_key = gtsam::Symbol('T', 1);
}  // namespace example

boost::shared_ptr<const ScrewJointBase> make_joint(gtsam::Pose3 cMp,
                                                   gtsam::Vector6 cScrewAxis) {
  // create links
  LinkParams link1_params, link2_params;
  link1_params.mass = 100;
  link1_params.name = "l1";
  link1_params.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  link1_params.wTl = gtsam::Pose3();
  link1_params.lTcom = gtsam::Pose3();
  link2_params = link1_params;
  link2_params.wTl = cMp.inverse();

  LinkSharedPtr l1 = boost::make_shared<Link>(Link(link1_params));
  LinkSharedPtr l2 = boost::make_shared<Link>(Link(link2_params));

  // create joint
  JointParams joint_params;
  joint_params.effort_type = JointEffortType::Actuated;
  joint_params.scalar_limits.value_lower_limit = -1.57;
  joint_params.scalar_limits.value_upper_limit = 1.57;
  joint_params.scalar_limits.value_limit_threshold = 0;
  gtsam::Pose3 wTj = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 jTccom = wTj.inverse() * l2->wTcom();
  gtsam::Vector6 jScrewAxis = jTccom.AdjointMap() * cScrewAxis;

  return boost::make_shared<const ScrewJointBase>(ScrewJointBase(
      1, "j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(), jScrewAxis));
}

// Test twistAccel factor for stationary case
TEST(TwistAccelFactor, error) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(cMp, screw_axis);

  // create factor
  TwistAccelFactor factor(example::twistKey, example::twistAccel_p_key,
                          example::twistAccel_c_key, example::qKey,
                          example::qVelKey, example::qAccelKey,
                          example::cost_model, joint);
  double q = M_PI / 4, qVel = 10, qAccel = 10;
  gtsam::Vector twist, twistAccel_p, twistAccel_c;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twistAccel_p = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twistAccel_c =
      (gtsam::Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twistAccel_p, twistAccel_c, q, qVel, qAccel);
  expected_errors << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::qAccelKey, qAccel);
  values.insert(example::twistKey, twist);
  values.insert(example::twistAccel_p_key, twistAccel_p);
  values.insert(example::twistAccel_c_key, twistAccel_c);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test twistAccel factor for stationary case
TEST(TwistAccelFactor, error_1) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis = (gtsam::Vector(6) << 0, 0, 1, 0, 1, 0).finished();

  auto joint = make_joint(cMp, screw_axis);

  TwistAccelFactor factor(example::twistKey, example::twistAccel_p_key,
                          example::twistAccel_c_key, example::qKey,
                          example::qVelKey, example::qAccelKey,
                          example::cost_model, joint);
  double q = 0, qVel = 0, qAccel = -9.8;
  gtsam::Vector6 twist, twistAccel_p, twistAccel_c;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twistAccel_p = (gtsam::Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  twistAccel_c = (gtsam::Vector(6) << 0, 0, -9.8, 0, 0, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twistAccel_p, twistAccel_c, q, qVel, qAccel);
  expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::qAccelKey, qAccel);
  values.insert(example::twistKey, twist);
  values.insert(example::twistAccel_p_key, twistAccel_p);
  values.insert(example::twistAccel_c_key, twistAccel_c);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
