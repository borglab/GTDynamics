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
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "gtdynamics/factors/ForwardKinematicsFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

Key key1 = PoseKey(0, 0), key2 = PoseKey(2, 0);

Robot getRobot() {
  // A three link robot with 2 revolute joints.
  Robot simple_rr = CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");
  return simple_rr;
}

TEST(ForwardKinematicsFactor, Constructor) {
  Robot robot = getRobot();
  JointValues joint_angles;

  for (auto&& joint : robot.joints()) {
    joint_angles[joint->name()] = 0.0;
  }

  auto model = noiseModel::Isotropic::Sigma(6, 0.1);
  ForwardKinematicsFactor(key1, key2, robot, "link_0", "link_2", joint_angles,
                          model);
}

TEST(ForwardKinematicsFactor, Error) {
  // simple_rr robot
  Robot robot = getRobot();
  JointValues joint_angles;

  // Lay the robot arm flat
  for (auto&& joint : robot.joints()) {
    joint_angles[joint->name()] = 0.0;
  }

  auto base_link = robot.links()[0]->name();
  // auto link1 = robot.links()[1]->name();
  auto end_link = robot.links()[2]->name();

  auto model = noiseModel::Isotropic::Sigma(6, 0.1);
  ForwardKinematicsFactor factor(key1, key2, robot, base_link, end_link,
                                 joint_angles, model);

  // Hand computed value from SDF file
  Pose3 bTl1(Rot3(), Point3(0, 0, 0.1)), bTl2(Rot3(), Point3(0, 0, 1.1));
  Vector error = factor.evaluateError(bTl1, bTl2);

  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));
}

TEST(ForwardKinematicsFactor, Jacobians) {
  // simple_rr robot
  Robot robot = getRobot();
  JointValues joint_angles;

  // Lay the robot arm flat
  for (auto&& joint : robot.joints()) {
    joint_angles[joint->name()] = 0.0;
  }

  auto base_link = robot.links()[0]->name();
  // auto link1 = robot.links()[1]->name();
  auto end_link = robot.links()[2]->name();

  auto model = noiseModel::Isotropic::Sigma(6, 0.1);
  ForwardKinematicsFactor factor(key1, key2, robot, base_link, end_link,
                                 joint_angles, model);

  Pose3 bTl1(Rot3(), Point3(0, 0, 0.1)), bTl2(Rot3(), Point3(0, 0, 1.1));

  Values values;
  values.insert<Pose3>(key1, bTl1);
  values.insert<Pose3>(key2, bTl2);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

TEST(ForwardKinematicsFactor, Movement) {
  // simple_rr robot
  Robot robot = getRobot();

  JointValues joint_angles;
  // Lay the robot arm flat
  for (auto&& joint : robot.joints()) {
    joint_angles[joint->name()] = 0.0;
  }
  joint_angles["joint_2"] = M_PI_2;

  auto base_link = robot.links()[0]->name();
  auto end_link = robot.links()[2]->name();

  auto fk_results = robot.forwardKinematics(
      joint_angles, joint_angles, base_link, robot.link(base_link)->lTcom());
  auto links = fk_results.first;

  auto model = noiseModel::Isotropic::Sigma(6, 0.1);
  ForwardKinematicsFactor factor(key1, key2, robot, base_link, end_link,
                                 joint_angles, model);

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

  Values values;
  values.insert<Pose3>(key1, bTl1);
  values.insert<Pose3>(key2, bTl2);
  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
