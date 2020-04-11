/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamicsGraph.cpp
 * @brief test forward and inverse dynamics factor graph
 * @Author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

#include "gtdynamics/dynamics/Simulator.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/Utils.h"

TEST(Simulate, simple_urdf) {
  using gtsam::assert_equal;
  using simple_urdf::my_robot, simple_urdf::gravity, simple_urdf::planar_axis;
  using std::vector;
  gtsam::Values joint_angles, joint_vels, torques;
  joint_angles.insert<double>(my_robot.getJointByName("j1")->getKey(), 0);
  joint_vels.insert<double>(my_robot.getJointByName("j1")->getKey(), 0);
  torques.insert<double>(my_robot.getJointByName("j1")->getKey(), 1);
  auto simulator = gtdynamics::Simulator(my_robot, joint_angles, joint_vels,
                                         gravity, planar_axis);

  int num_steps = 1 + 1;
  double dt = 1;
  vector<gtsam::Values> torques_seq(num_steps, torques);
  auto results = simulator.simulate(torques_seq, dt);

  int t = 1;
  gtsam::Values qs =
      gtdynamics::DynamicsGraph::jointAngles(my_robot, results, t);
  gtsam::Values vs = gtdynamics::DynamicsGraph::jointVels(my_robot, results, t);
  gtsam::Values as =
      gtdynamics::DynamicsGraph::jointAccels(my_robot, results, t);

  double acceleration = 0.0625;
  double expected_qAccel = acceleration;
  double expected_qVel = acceleration * dt;
  double expected_qAngle = acceleration * 0.5 * dt * dt;
  EXPECT(assert_equal(expected_qAngle, qs.at<double>(my_robot.getJointByName("j1")->getKey())));
  EXPECT(assert_equal(expected_qVel, vs.at<double>(my_robot.getJointByName("j1")->getKey())));
  EXPECT(assert_equal(expected_qAccel, as.at<double>(my_robot.getJointByName("j1")->getKey())));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
