/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTwistFactor.cpp
 * @brief Test twist factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#include "gtdynamics/universal_robot/ScrewJointBase.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/factors/TwistFactor.h"

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key twist_p_key = gtsam::Symbol('V', 1),
           twist_c_key = gtsam::Symbol('V', 2), qKey = gtsam::Symbol('q', 0),
           qVelKey = gtsam::Symbol('j', 0);
}  // namespace example

ScrewJointBaseConstSharedPtr make_joint(gtsam::Pose3 cMp,
                                        gtsam::Vector6 cScrewAxis) {
  // create links
  Link::Params link1_params, link2_params;
  link1_params.mass = 100;
  link1_params.name = "l1";
  link1_params.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  link1_params.wTl = gtsam::Pose3();
  link1_params.lTcom = gtsam::Pose3();
  link2_params = link1_params;
  link2_params.wTl = cMp.inverse();

  LinkSharedPtr l1 = std::make_shared<Link>(Link(link1_params));
  LinkSharedPtr l2 = std::make_shared<Link>(Link(link2_params));

  // create joint
  ScrewJointBase::Parameters joint_params;
  joint_params.effort_type = Joint::EffortType::Actuated;
  joint_params.scalar_limits.value_lower_limit = -1.57;
  joint_params.scalar_limits.value_upper_limit = 1.57;
  joint_params.scalar_limits.value_limit_threshold = 0;
  gtsam::Pose3 wTj = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 jTccom = wTj.inverse() * l2->wTcom();
  gtsam::Vector6 jScrewAxis = jTccom.AdjointMap() * cScrewAxis;

  return std::make_shared<const ScrewJointBase>(
      ScrewJointBase("j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(),
      jScrewAxis));
}

// Test twist factor for stationary case
TEST(TwistFactor, error) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(cMp, screw_axis);

  TwistFactor factor(example::twist_p_key, example::twist_c_key, example::qKey,
                     example::qVelKey, example::cost_model, joint);
  double q = M_PI / 4, qVel = 10;
  gtsam::Vector twist_p, twist_c;
  twist_p = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_c =
      (gtsam::Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(twist_p, twist_c, q, qVel);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
