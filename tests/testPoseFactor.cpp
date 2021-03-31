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
#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key wTp_key = internal::PoseKey(1), wTc_key = internal::PoseKey(2),
           q_key = internal::JointAngleKey(1);
}  // namespace example

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);

  // Create factor
  PoseFactor factor(example::wTp_key, example::wTc_key, example::q_key,
                    example::cost_model, joint);

  // call unwhitenedError
  gtsam::Values values;
  InsertPose(&values, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  InsertPose(&values, 2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0)));
  InsertJointAngle(&values, 1, 0.0);
  auto actual_errors = factor.unwhitenedError(values);

  // check value
  auto expected_errors = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
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
  PoseFactor factor(example::wTp_key, example::wTc_key, example::q_key,
                    example::cost_model, joint);

  // check prediction at zero joint angle
  {
    gtsam::Values values;
    InsertPose(&values, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
    InsertPose(&values, 2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0)));
    InsertJointAngle(&values, 1, 0.0);
    EXPECT(assert_equal(gtsam::Z_6x1, factor.unwhitenedError(values), 1e-6));
  }

  // check prediction at half PI
  {
    gtsam::Values values;
    InsertPose(&values, 1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
    InsertPose(&values, 2,
               gtsam::Pose3(gtsam::Rot3::Rz(M_PI / 2), gtsam::Point3(2, 1, 0)));
    InsertJointAngle(&values, 1, M_PI / 2);
    EXPECT(assert_equal(gtsam::Z_6x1, factor.unwhitenedError(values), 1e-6));
  }
}

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {
  // Evaluate PoseFunctor on an RR link.
  using simple_urdf_zero_inertia::robot;

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");
  auto j1 = boost::dynamic_pointer_cast<gtdynamics::ScrewJointBase>(
      robot.joint("j1"));

  gtsam::Vector6 screw_axis =
      (gtsam::Vector(6) << 1, 0, 0, 0, -1, 0).finished();
  gtsam::Pose3 cMp = j1->relativePoseOf(l1, 0.0);
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::wTp_key, example::wTc_key, example::q_key,
                    example::cost_model, joint);

  // unwhitenedError
  gtsam::Values values;
  InsertPose(&values, 1, gtsam::Pose3());
  InsertPose(&values, 2, j1->relativePoseOf(l2, M_PI / 4));
  InsertJointAngle(&values, 1, M_PI / 4);
  EXPECT(assert_equal(gtsam::Z_6x1, factor.unwhitenedError(values), 1e-6));
}

// Test non-zero cMp rotation case
TEST(PoseFactor, nonzero_rest) {
  // Create factor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3::Rx(1), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto joint = make_joint(cMp, screw_axis);
  PoseFactor factor(example::wTp_key, example::wTc_key, example::q_key,
                    example::cost_model, joint);

  double jointAngle;
  gtsam::Pose3 pose_p, pose_c;
  // zero joint angle
  jointAngle = 0;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  // Make sure linearization is correct
  {
    gtsam::Values values;
    InsertPose(&values, 1, pose_p);
    InsertPose(&values, 2, pose_c);
    InsertJointAngle(&values, 1, jointAngle);
    double diffDelta = 1e-7;
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
  }

  // half PI
  {
    gtsam::Values values;
    jointAngle = M_PI / 2;
    pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    pose_c = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
    InsertPose(&values, 1, pose_p);
    InsertPose(&values, 2, pose_c);
    InsertJointAngle(&values, 1, jointAngle);
    double diffDelta = 1e-7;
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
