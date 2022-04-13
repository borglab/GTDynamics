/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testForwardKinematicsFactor.cpp
 * @brief Test ForwardKinematicsFactor.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ForwardKinematicsFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

const size_t i1 = 0, i2 = 2;
const Key key1 = gtdynamics::PoseKey(i1), key2 = gtdynamics::PoseKey(i2);

auto kModel = noiseModel::Isotropic::Sigma(6, 0.1);

auto robot = simple_rr::getRobot();

gtsam::Values zeroValues() {
  gtsam::Values joint_angles;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&joint_angles, joint->id(), 0.0);
  }
  return joint_angles;
}

TEST(ForwardKinematicsFactor, Constructor) {
  gtsam::Values joint_angles = zeroValues();

  ForwardKinematicsFactor(key1, key2, robot, "link_0", "link_2", joint_angles,
                          kModel);
}

TEST(ForwardKinematicsFactor, Error) {
  gtsam::Values joint_angles = zeroValues();

  auto base_link = robot.links()[0];
  auto end_link = robot.links()[2];

  ForwardKinematicsFactor factor(key1, key2, robot, base_link->name(),
                                 end_link->name(), joint_angles, kModel);

  // Hand computed value from SDF file
  Pose3 bTl1(Rot3(), Point3(0, 0, 0.1)), bTl2(Rot3(), Point3(0, 0, 1.1));
  Vector error = factor.evaluateError(bTl1, bTl2);

  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));
}

TEST(ForwardKinematicsFactor, Jacobians) {
  gtsam::Values joint_angles = zeroValues();

  auto base_link = robot.links()[0];
  auto end_link = robot.links()[2];

  ForwardKinematicsFactor factor(key1, key2, robot, base_link->name(),
                                 end_link->name(), joint_angles, kModel);

  Pose3 bTl1(Rot3(), Point3(0, 0, 1)), bTl2(Rot3(), Point3(0, 0, 2));

  Values values;
  InsertPose(&values, i1, bTl1);
  InsertPose(&values, i2, bTl2);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

TEST(ForwardKinematicsFactor, Movement) {
  Values values;

  // Lay the robot arm flat
  InsertJointAngle(&values, 1, M_PI_2);
  Values values_for_jacobians;

  auto base_link = robot.links()[0];
  auto end_link = robot.links()[2];

  ForwardKinematicsFactor factor(key1, key2, robot, base_link->name(),
                                 end_link->name(), values, kModel);

  Pose3 bTl1(Rot3(), Point3(0, 0, 0.1));
  // We rotated the last link by 90 degrees
  // Since the joint is originally at 0.8, the CoM of link_2 will have z=0.8,
  // and the extra 0.3 moves to the x-axis.
  Pose3 bTl2(Rot3(0, 0, 1,  //
                  0, 1, 0,  //
                  -1, 0, 0),
             Point3(0.3, 0, 0.8));
  Vector error = factor.evaluateError(bTl1, bTl2);
  // Check end pose error
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));

  // Check Jacobians
  InsertPose(&values_for_jacobians, i1, bTl1);
  InsertPose(&values_for_jacobians, i2, bTl2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values_for_jacobians, 1e-7, 1e-5);
}

TEST(ForwardKinematicsFactor, ArbitraryTime) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  std::string base_link = "l1", end_link = "l2";

  size_t t = 81;
  Values joint_angles;
  InsertJointAngle(&joint_angles, robot.joint("j1")->id(), t, M_PI_2);

  ForwardKinematicsFactor factor(key1, key2, robot, base_link, end_link,
                                 joint_angles, kModel, t);

  Pose3 actual = factor.measured();
  Values fk = robot.forwardKinematics(joint_angles, t, base_link);
  Pose3 wTl1 = Pose(fk, robot.link("l1")->id(), t),
        wTl2 = Pose(fk, robot.link("l2")->id(), t);
  Pose3 expected = wTl1.between(wTl2);
  EXPECT(assert_equal(expected, actual));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
