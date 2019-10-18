/**
 * @file testJumpingRobotDynamics.cpp
 * @brief test forward dynamics of robotic manipulator with closed kinematic
 * loop
 * @Author: Yetong Zhang
 */

#include <Arm.h>
#include <URDFLink.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {

int num_links = 5;
double m1 = 0.31;
double m2 = 0.28;
double m3 = 0.54;
double link_radius = 0.02;
double l = 0.55;

Arm<URDF_Link> getRobot() {
  Vector link_length = Vector::Zero(num_links);
  Vector link_mass = Vector::Zero(num_links);
  link_length << l, l, l, l, l;
  link_mass << m1, m2, m3, m2, m1;
  vector<gtsam::Matrix3> link_inertia;
  for (int i = 0; i < num_links; i++) {
    gtsam::Matrix3 inertia;
    double principal_inertia1 =
        1.0 / 2 * link_mass(i) * std::pow(link_radius, 2);
    double principal_inertia2 =
        1.0 / 12 * link_mass(i) *
        (3 * std::pow(link_radius, 2) + std::pow(link_length(i), 2));

    inertia << principal_inertia1, 0, 0, 0, principal_inertia2, 0, 0, 0,
        principal_inertia2;
    link_inertia.push_back(inertia);
  }

  vector<URDF_Link> urdf_jump = {
      URDF_Link(Pose3(Rot3::Rz(M_PI / 2), Point3(link_length(2) / 2, 0, 0)),
                Vector3(0, 0, 1), 'R', link_mass(0),
                Pose3(Rot3(), Point3(link_length(0) / 2, 0, 0)),
                link_inertia[0], false, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(0), Point3(link_length(0), 0, 0)),
                Vector3(0, 0, 1), 'R', link_mass(1),
                Pose3(Rot3(), Point3(link_length(1) / 2, 0, 0)),
                link_inertia[1], true, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(M_PI / 2), Point3(link_length(1), 0, 0)),
                Vector3(0, 0, 1), 'R', link_mass(2),
                Pose3(Rot3(), Point3(link_length(2) / 2, 0, 0)),
                link_inertia[2], true, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(M_PI / 2), Point3(link_length(2), 0, 0)),
                Vector3(0, 0, 1), 'R', link_mass(3),
                Pose3(Rot3(), Point3(link_length(3) / 2, 0, 0)),
                link_inertia[3], true, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(0), Point3(link_length(3), 0, 0)),
                Vector3(0, 0, 1), 'R', link_mass(4),
                Pose3(Rot3(), Point3(link_length(4) / 2, 0, 0)),
                link_inertia[4], true, 0, 0)};

  Pose3 base = Pose3();
  Pose3 tool = Pose3(Rot3(), Point3(2, 0, 0));

  // get screw_axis for loop closure
  auto screw_axis =
      unit_twist(Vector3(0, 0, 1), Vector3(-link_length(4) / 2, 0, 0));
  return Arm<URDF_Link>(urdf_jump, base, tool, screw_axis, false, 0, 0);
}
auto robot = getRobot();
auto dof = robot.numLinks() + 1;
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof);
Vector torque_ID = Vector::Zero(dof);
}  // namespace example

/* ************************************************************************/
// Test forward dynamics of jumping robot
TEST(JumpingRobot, forward_dynamics) {
  double torque3 = 1;
  double torque2 = 0.5;
  double theta = 30.0 / 180.0 * M_PI;

  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();
  initialJointAngles << -theta, 2 * theta, -theta, -theta, 2 * theta, -theta;
  known_torque << 0, torque2, torque3, torque3, torque2, 0;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, known_torque,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_accelerations =
      example::robot.extractJointAcceleraions(results, example::dof);

  auto expected_accelerations = Vector(6);
  double acc =
      (torque3 - torque2 * 2 - (0.5 * example::m1 + 1.5 * example::m2 + 1.0 * example::m3) *
                     9.8 * example::l * std::sin(theta)) /
      (std::pow(example::l, 2) *
           (1.0 / 4 * example::m1 +
            (1.0 / 4 + 2 * std::pow(std::sin(theta), 2)) * example::m2 +
            2 * std::pow(std::sin(theta), 2) * example::m3) +
       (std::pow(example::l, 2) + 3 * std::pow(example::link_radius, 2)) *
           (1.0 / 12 * example::m1 + 1.0 / 12 * example::m2));
  expected_accelerations << acc, -2 * acc, acc, acc, -2 * acc, acc;
  EXPECT(assert_equal(expected_accelerations, actual_accelerations, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
