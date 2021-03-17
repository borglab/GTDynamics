/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPoseFactor.cpp
 * @brief Test forward kinematics factor.
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

#include <iostream>

#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key pose_p_key = gtsam::Symbol('p', 1),
           pose_c_key = gtsam::Symbol('p', 2), qKey = gtsam::Symbol('q', 0);
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
      "j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(), jScrewAxis));
}

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  double jointAngle = 0;

  // Create factor
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, joint);

  // call evaluateError
  gtsam::Pose3 pose_p(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Pose3 pose_c(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  auto actual_errors = factor.evaluateError(pose_p, pose_c, jointAngle);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test breaking case
TEST(PoseFactor, breaking) {
  // create functor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, joint);
  double jointAngle;
  gtsam::Pose3 pose_p, pose_c;
  // check prediction at zero joint angle
  jointAngle = 0;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  EXPECT(assert_equal(gtsam::Vector6::Zero(),
                      factor.evaluateError(pose_p, pose_c, jointAngle), 1e-6));

  // check prediction at half PI
  jointAngle = M_PI / 2;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
  EXPECT(assert_equal(gtsam::Vector6::Zero(),
                      factor.evaluateError(pose_p, pose_c, jointAngle), 1e-6));
}

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {
  // Evaluate PoseFunctor on an RR link.
  using simple_urdf_zero_inertia::my_robot;

  gtsam::Pose3 base_pose =
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));

  double joint_angle = M_PI / 4;

  auto l2 = my_robot.link("l2");
  auto j1 = boost::dynamic_pointer_cast<gtdynamics::ScrewJointBase>(
      my_robot.joint("j1"));

  gtsam::Vector6 screw_axis =
      (gtsam::Vector(6) << 1, 0, 0, 0, -1, 0).finished();
  gtsam::Pose3 cMp = j1->transformTo(l2, 0.0);
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, joint);

  EXPECT(assert_equal(
      gtsam::Vector6::Zero(),
      factor.evaluateError(base_pose, j1->transformFrom(l2, joint_angle),
                           joint_angle),
      1e-6));
}

// Test non-zero cMp rotation case
TEST(PoseFactor, nonzero_rest) {
  // Create factor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3::Rx(1), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, joint);

  double jointAngle;
  gtsam::Pose3 pose_p, pose_c;
  // zero joint angle
  jointAngle = 0;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // half PI
  jointAngle = M_PI / 2;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
  values.update(example::pose_p_key, pose_p);
  values.update(example::pose_c_key, pose_c);
  values.update(example::qKey, jointAngle);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
