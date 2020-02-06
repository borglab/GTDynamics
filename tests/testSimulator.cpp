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

#include <RobotModels.h>
#include <Simulator.h>
#include <Robot.h>
#include <gtsam/slam/PriorFactor.h>
#include <utils.h>

#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

TEST(Simulate, simple_urdf)
{
  using simple_urdf::my_robot, simple_urdf::gravity, simple_urdf::planar_axis;
  using std::vector;
  using gtsam::assert_equal;
  Robot::JointValues joint_angles, joint_vels, torques;
  joint_angles["j1"] = 0;
  joint_vels["j1"] = 0;
  torques["j1"] = 1;
  auto simulator = gtdynamics::Simulator(my_robot, joint_angles, joint_vels, gravity, planar_axis);

  int num_steps = 1 + 1;
  double dt = 1;
  vector<Robot::JointValues> torques_seq(num_steps, torques);
  auto results = simulator.simulate(torques_seq, dt);

  int t = 1;
  gtsam::Vector qs = gtdynamics::DynamicsGraph::jointAngles(my_robot, results, t);
  gtsam::Vector vs = gtdynamics::DynamicsGraph::jointVels(my_robot, results, t);
  gtsam::Vector as = gtdynamics::DynamicsGraph::jointAccels(my_robot, results, t);

  double acceleration = 0.0625;
  double expected_qAccel = acceleration;
  double expected_qVel = acceleration * dt;
  double expected_qAngle = acceleration * 0.5 * dt * dt;
  EXPECT(assert_equal(expected_qAngle, qs[0]));
  EXPECT(assert_equal(expected_qVel, vs[0]));
  EXPECT(assert_equal(expected_qAccel, as[0]));
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
