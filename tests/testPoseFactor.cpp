/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPoseFactor.cpp
 * @brief Test forward kinematics factor.
 * @Author: Frank Dellaert and Mandy Xie
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
#include <memory>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"

using namespace gtdynamics; 
using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key pose_i_key = gtsam::Symbol('p', 1),
           pose_j_key = gtsam::Symbol('p', 2), qKey = gtsam::Symbol('q', 0);
}  // namespace example

ScrewJointBaseConstSharedPtr make_joint(gtsam::Pose3 jMi,
                                        gtsam::Vector6 cScrewAxis) {
  // create links
  Link::Params link1_params, link2_params;
  link1_params.mass = 100;
  link1_params.name = "l1";
  link1_params.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  link1_params.wTl = gtsam::Pose3();
  link1_params.lTcom = gtsam::Pose3();
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
  gtsam::Pose3 wTj = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 jTccom = wTj.inverse() * l2->wTcom();
  gtsam::Vector6 jScrewAxis = jTccom.AdjointMap() * cScrewAxis;

  return std::make_shared<const ScrewJointBase>(
      ScrewJointBase("j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(),
      jScrewAxis));
}

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  gtsam::Pose3 jMi = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(jMi, screw_axis);
  double jointAngle = 0;

  // Create factor
  PoseFactor<ScrewJointBase> factor(example::pose_i_key, example::pose_j_key,
                                    example::qKey, example::cost_model, joint);

  // call evaluateError
  gtsam::Pose3 pose_i(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      pose_j(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  auto actual_errors = factor.evaluateError(pose_i, pose_j, jointAngle);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test breaking case
TEST(PoseFactor, breaking) {
  // create functor
  gtsam::Pose3 jMi = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(jMi, screw_axis);
  PoseFactor<ScrewJointBase> factor(example::pose_i_key, example::pose_j_key,
                                example::qKey, example::cost_model, joint);
  double jointAngle;
  gtsam::Pose3 pose_i, pose_j;

  // check prediction at zero joint angle
  jointAngle = 0;
  pose_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_j = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  EXPECT(assert_equal(gtsam::Vector6::Zero(),
                      factor.evaluateError(pose_i, pose_j, jointAngle), 1e-6));

  // check prediction at half PI
  jointAngle = M_PI / 2;
  pose_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_j = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
  EXPECT(assert_equal(gtsam::Vector6::Zero(),
                      factor.evaluateError(pose_i, pose_j, jointAngle), 1e-6));
}

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {
  // Evaluate PoseFunctor on an RR link.
  using simple_urdf_zero_inertia::my_robot;

  gtsam::Pose3 base_pose =
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));

  double joint_angle = M_PI / 4;

  auto l2 = my_robot.getLinkByName("l2");
  auto j1 = std::dynamic_pointer_cast<gtdynamics::RevoluteJoint>(
      my_robot.getJointByName("j1"));
  gtsam::Vector6 screw_axis =
      (gtsam::Vector(6) << 1, 0, 0, 0, -1, 0).finished();
  gtsam::Pose3 jMi = j1->transformTo(l2);
  auto joint = make_joint(jMi, screw_axis);
  PoseFactor<ScrewJointBase> factor(example::pose_i_key, example::pose_j_key,
                                example::qKey, example::cost_model, joint);

  EXPECT(assert_equal(
      gtsam::Vector6::Zero(),
      factor.evaluateError(base_pose, j1->transformFrom(l2, joint_angle),
                           joint_angle),
      1e-6));
}

// Test non-zero jMi rotation case
TEST(PoseFactor, nonzero_rest) {
  // Create factor
  gtsam::Pose3 jMi = gtsam::Pose3(gtsam::Rot3::Rx(1), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(jMi, screw_axis);
  PoseFactor<ScrewJointBase> factor(example::pose_i_key, example::pose_j_key,
                                    example::qKey, example::cost_model, joint);

  double jointAngle;
  gtsam::Pose3 pose_i, pose_j;
  // zero joint angle
  jointAngle = 0;
  pose_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_j = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // half PI
  jointAngle = M_PI / 2;
  pose_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_j = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
  values.update(example::pose_i_key, pose_i);
  values.update(example::pose_j_key, pose_j);
  values.update(example::qKey, jointAngle);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
