/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTorqueFactor.cpp
 * @brief Test torque factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Pose3, gtsam::Vector6, gtsam::Vector3, gtsam::Rot3, gtsam::Point3;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
gtsam::Key torque_key = gtsam::Symbol('t', 1),
           wrench_key = gtsam::Symbol('F', 1);
}  // namespace example

ScrewJointBaseConstSharedPtr make_joint(Pose3 jMi,
                                        Vector6 cScrewAxis) {
  // create links
  Link::Params link1_params, link2_params;
  link1_params.mass = 100;
  link1_params.name = "l1";
  link1_params.inertia = Vector3(3, 2, 1).asDiagonal();
  link1_params.wTl = Pose3();
  link1_params.lTcom = Pose3();
  link2_params = link1_params;
  link2_params.wTl = jMi.inverse();

  LinkSharedPtr l1 = std::make_shared<Link>(Link(link1_params));
  LinkSharedPtr l2 = std::make_shared<Link>(Link(link2_params));

  // create joint
  ScrewJointBase::Parameters joint_params;
  joint_params.effort_type = JointEffortType::Actuated;
  joint_params.joint_lower_limit = -1.57;
  joint_params.joint_upper_limit = 1.57;
  joint_params.joint_limit_threshold = 0;
  Pose3 wTj = Pose3(Rot3(), Point3(0, 0, 2));
  Pose3 jTccom = wTj.inverse() * l2->wTcom();
  Vector6 jScrewAxis = jTccom.AdjointMap() * cScrewAxis;

  return std::make_shared<const ScrewJointBase>(
      ScrewJointBase("j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(),
      jScrewAxis));
}

// Test Torque factor for stationary case
TEST(TorqueFactor, error) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(0, 0, -2)); // doesn't matter
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(kMj, screw_axis);

  TorqueFactor factor(example::wrench_key, example::torque_key,
                                      example::cost_model, joint);
  double torque = 20;
  gtsam::Vector wrench = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  gtsam::Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(wrench, torque);
  expected_errors = gtsam::Vector1(0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::torque_key, torque);
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
