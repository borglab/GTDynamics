/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSimulator.cpp
 * @brief test Simulator class
 * @author: Yetong Zhang
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
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;

TEST(Simulate, simple_urdf) {
  using gtsam::assert_equal;
  using simple_urdf::robot, simple_urdf::gravity, simple_urdf::planar_axis;
  using std::vector;
  gtsam::Values initial_values, torques;
  robot.print();
  InsertJointAngle(&initial_values, 0, 0.0);
  InsertTorque(&torques, 0, 1.0);

  Simulator simulator(robot, initial_values, gravity, planar_axis);

  int num_steps = 1 + 1;
  double dt = 1;
  vector<gtsam::Values> torques_seq(num_steps, torques);
  auto results = simulator.simulate(torques_seq, dt);

  // GTD_PRINT(results);

  double acceleration = 0.0625;
  double expected_qAccel = acceleration;
  double expected_qVel = acceleration * dt;
  double expected_qAngle = acceleration * 0.5 * dt * dt;
  EXPECT(assert_equal(expected_qAngle, JointAngle(results, 0)));
  EXPECT(assert_equal(expected_qVel, JointVel(results, 0)));
  EXPECT(assert_equal(expected_qAccel, JointAccel(results, 0)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
