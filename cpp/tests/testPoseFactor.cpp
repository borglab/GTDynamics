/**
 * @file  testPoseFactor.cpp
 * @brief test forward kinematics factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <DHLink.h>
#include <Arm.h>
#include <PoseFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);
Key pose_i_key = Symbol('p', 1), pose_j_key = Symbol('p', 2),
    qKey = Symbol('q', 0);
}  // namespace example

/**
 * Test twist factor for stationary case
 */
TEST(PoseFactor, error) {
  // create functor
  Pose3 jMi = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis<< 0, 0, 1, 0, 1, 0;
  PoseFunctor predictPose(jMi, screw_axis);

  // check prediction
  double jointAngle = 0;
  Pose3 pose_i(Rot3(), Point3(1, 0, 0)), pose_j(Rot3(), Point3(3, 0, 0));
  EXPECT(assert_equal(pose_j, predictPose(pose_i, jointAngle), 1e-6));

  // Create factor
  PoseFactor factor(example::pose_i_key, example::pose_j_key, example::qKey,
                    example::cost_model, jMi, screw_axis);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose_i, pose_j, jointAngle);

  // check value
  auto expected_errors = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::pose_i_key, pose_i);
  values.insert(example::pose_j_key, pose_j);
  values.insert(example::qKey, jointAngle);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test breaking case
 */
TEST(PoseFactor, breaking) {
  // create functor
  Pose3 jMi = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  PoseFunctor predictPose(jMi, screw_axis);

  double jointAngle;
  Pose3 pose_i, pose_j;
  // check prediction at zero joint angle
  jointAngle = 0;
  pose_i = Pose3(Rot3(), Point3(1, 0, 0));
  pose_j = Pose3(Rot3(), Point3(3, 0, 0));
  EXPECT(assert_equal(pose_j, predictPose(pose_i, jointAngle), 1e-6));

  // check prediction at half PI
  jointAngle = M_PI / 2;
  pose_i = Pose3(Rot3(), Point3(1, 0, 0));
  pose_j = Pose3(Rot3::Rz(jointAngle), Point3(2, 1, 0));
  EXPECT(assert_equal(pose_j, predictPose(pose_i, jointAngle), 1e-6));
}

/**
 * Test breaking case for rr link
 */
TEST(PoseFactor, breaking_rr) {
  // RR link example
  vector<DH_Link> dh_rr = {DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Z_3x3),
                           DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Z_3x3)};
  auto robot = Arm<DH_Link>(dh_rr);
  Pose3 pose_goal(Pose3(Rot3::Rz(M_PI / 2), Point3(0, 4, 0)));
  auto dof = robot.numLinks();

  // get robot jTi list at rest
  auto jMi = robot.jTi_list(Vector::Zero(dof));
  // get base pose in world frame
  auto basePose = robot.base();
  // get robot screwAxes for all links
  auto screwAxes = robot.screwAxes();
  // joint angles value
  Vector2 jointAngles(M_PI / 2, 0);
  // get link COM expected poses
  auto expectedPoses = robot.comFrames(jointAngles);

  Pose3 pose_i = basePose;

  for (int k = 0; k < dof; ++k) {
    // create functor
    PoseFunctor predictPose(jMi[k], screwAxes[k]);
    EXPECT(assert_equal(expectedPoses[k], predictPose(pose_i, jointAngles[k]), 1e-6));
    pose_i = expectedPoses[k];
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
