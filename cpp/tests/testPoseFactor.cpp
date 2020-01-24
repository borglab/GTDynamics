/**
 * @file  testPoseFactor.cpp
 * @brief test forward kinematics factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <PoseFactor.h>

#include <RobotModels.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

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

// Test twist factor for stationary case
TEST(PoseFactor, error) {
  // create functor
  Pose3 jMi = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
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

// Test breaking case
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

// Test breaking case for rr link
TEST(PoseFactor, breaking_rr) {

  // Evaluate PoseFunctor on an RR link.
  using namespace simple_urdf_zero_inertia;

  gtsam::Pose3 base_pose = gtsam::Pose3(
    gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));
  
  double joint_angle = M_PI / 4;

  gtsam::Vector6 screw_axis = my_robot.screwAxes()["j1"];
  gtsam::Pose3 jMi = my_robot.getJointByName("j1")->McpCom();

  PoseFunctor predictPose(jMi, screw_axis);

  EXPECT(assert_equal(
    my_robot.getJointByName("j1")->MpcCom(joint_angle), 
    predictPose(base_pose, joint_angle),
    1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
