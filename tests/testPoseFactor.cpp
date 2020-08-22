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

#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key pose_p_key = gtsam::Symbol('p', 1),
           pose_c_key = gtsam::Symbol('p', 2),
           qKey = gtsam::Symbol('q', 0);
}  // namespace example

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-2, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  PoseFunctor predictPose(cMp, screw_axis);

  // check prediction
  double jointAngle = 0;
  gtsam::Pose3 pose_p(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      pose_c(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  EXPECT(assert_equal(pose_c, predictPose(pose_p, jointAngle), 1e-6));

  // Create factor
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, cMp, screw_axis);

  // call evaluateError
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
  PoseFunctor predictPose(cMp, screw_axis);

  double jointAngle;
  gtsam::Pose3 pose_p, pose_c;
  // check prediction at zero joint angle
  jointAngle = 0;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 0, 0));
  EXPECT(assert_equal(pose_c, predictPose(pose_p, jointAngle), 1e-6));

  // check prediction at half PI
  jointAngle = M_PI / 2;
  pose_p = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_c = gtsam::Pose3(gtsam::Rot3::Rz(jointAngle), gtsam::Point3(2, 1, 0));
  EXPECT(assert_equal(pose_c, predictPose(pose_p, jointAngle), 1e-6));
}

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {
  // Evaluate PoseFunctor on an RR link.
  using simple_urdf_zero_inertia::my_robot;

  gtsam::Pose3 base_pose =
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));

  double joint_angle = M_PI / 4;

  auto l2 = my_robot.getLinkByName("l2");
  auto j1 = my_robot.getJointByName("j1");
  gtsam::Vector6 screw_axis =
      (gtsam::Vector(6) << 1, 0, 0, 0, -1, 0).finished();
  gtsam::Pose3 cMp = j1->transformTo(l2);
  PoseFunctor predictPose(cMp, screw_axis);

  EXPECT(assert_equal(j1->transformFrom(l2, joint_angle),
                      predictPose(base_pose, joint_angle), 1e-6));
}

TEST(PoseFactor, gradient) {
  using namespace gtsam;
  // create functor
  Pose3 cMp = Pose3(Rot3(), Point3(-0.5, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 0.25, 0;
  PoseFunctor predictPose(cMp, screw_axis);

  // check prediction
  double jointAngle = 2.928282;
  Matrix Ri = (Matrix(3, 3) << -0.999467542665, -0.0326286861324, 0,
               0.0326286861324, -0.999467542665, 0, 0, 0, 1)
                  .finished();
  Matrix Rj = (Matrix(3, 3) << 0.983719675006, -0.179709768814, 0,
               0.179709768814, 0.983719675006, 0, 0, 0, 1)
                  .finished();
  Rot3 rot3i(Ri), rot3j(Rj);
  Pose3 pose_p(rot3i, Point3(-0.249561117759, 0.000237810877085, 0)),
      pose_c(rot3j, Point3(-0.245265857966, -0.050829234479, 0));

  // Create factor
  PoseFactor factor(example::pose_p_key, example::pose_c_key, example::qKey,
                    example::cost_model, cMp, screw_axis);

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::pose_p_key, pose_p);
  values.insert(example::pose_c_key, pose_c);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
