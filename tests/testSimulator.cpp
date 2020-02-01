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
#include <UniversalRobot.h>
#include <gtsam/slam/PriorFactor.h>
#include <utils.h>

#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

// TEST(SimulatorFD, jumping_robot)
// {
//   using namespace jumping_robot;
//   auto simulator = Simulator(my_robot, joint_angles, joint_vels, gravity,
//   planar_axis);

//   double torque3 = 0;
//   double torque2 = 0.5;
//   Vector torques = (Vector(6) << 0, torque2, torque3, torque3, torque2,
//   0).finished(); simulator.forwardDynamics(torques); Vector actual_qAccel =
//   simulator.getJointAccelerations();

//   // check acceleration
//   auto expected_qAccel = Vector(6);
//   double m1 = 0.31;
//   double m2 = 0.28;
//   double m3 = 0.54;
//   double link_radius = 0.02;
//   double l = 0.55;
//   double theta = 0.0 / 180.0 * M_PI;
//   double acc =
//       (torque3 - torque2 * 2 - (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * 9.8 * l *
//       std::sin(theta)) / (std::pow(l, 2) * (1.0 / 4 * m1 + (1.0 / 4 + 2 *
//       std::pow(std::sin(theta), 2)) * m2 + 2 * std::pow(std::sin(theta), 2) *
//       m3) +
//        (std::pow(l, 2) + 3 * std::pow(link_radius, 2)) * (1.0 / 12 * m1 + 1.0
//        / 12 * m2));
//   expected_qAccel << acc, -2 * acc, acc, acc, -2 * acc, acc;
//   EXPECT(assert_equal(expected_qAccel, actual_qAccel));
// }

// TEST(Simulate, simple_urdf)
// {
//   using namespace simple_urdf;
//   auto simulator = Simulator(my_robot, joint_angles, joint_vels, gravity,
//   planar_axis); Vector torques = (Vector(1) << 1).finished(); int num_steps =
//   1 + 1; double dt = 1; vector<Vector> torques_seq(num_steps, torques); auto
//   results = simulator.simulate(torques_seq, dt); vector<Vector>
//   joint_angles_seq; vector<Vector> joint_vels_seq; vector<Vector>
//   joint_accels_seq; for (int t=0; t<num_steps; t++) {
//     joint_angles_seq.emplace_back(DynamicsGraphBuilder::jointAngles(my_robot,
//     results, t));
//     joint_vels_seq.emplace_back(DynamicsGraphBuilder::jointVels(my_robot,
//     results, t));
//     joint_accels_seq.emplace_back(DynamicsGraphBuilder::jointAccels(my_robot,
//     results, t));
//     // cout << t << "\t" << joint_angles_seq.back() << "\t" <<
//     joint_vels_seq.back() << "\t" << joint_accels_seq.back() << "\n";
//   }

//   double acceleration = 0.0625;
//   Vector expected_qAccel = (Vector(1) << acceleration).finished();
//   Vector expected_qVel = (Vector(1) << acceleration * 1.0).finished();
//   Vector expected_qAngle = (Vector(1) << acceleration * 0.5 * 1.0
//   * 1.0).finished(); EXPECT(assert_equal(expected_qAccel,
//   joint_accels_seq.back())); EXPECT(assert_equal(expected_qVel,
//   joint_vels_seq.back())); EXPECT(assert_equal(expected_qAngle,
//   joint_angles_seq.back()));
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
