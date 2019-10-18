/**
 * @file testSimpleLoopDynamics.cpp
 * @brief test dynamics of robotic manipulator with closed kinematic loop
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

static const double HALF_PI = M_PI / 2;

namespace example {

Arm<URDF_Link> getRobot() {
  gtsam::Matrix3 inertia;
  double principal_inertia = 1.0 / 12 * 2 * 2;
  inertia << 0, 0, 0, 0, principal_inertia, 0, 0, 0, principal_inertia;

  vector<URDF_Link> urdf_rrr = {
      URDF_Link(Pose3(Rot3::Rz(HALF_PI), Point3(-1, 0, 0)), Vector3(0, 0, 1),
                'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), inertia, true, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1),
                'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), inertia, false, 0, 0),
      URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1),
                'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), inertia, false, 0, 0)};
  Pose3 base = Pose3();
  Pose3 tool = Pose3(Rot3(), Point3(2, 0, 0));

  // get screw_axis for loop closure
  auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(1, 0, 0));
  return Arm<URDF_Link>(urdf_rrr, base, tool, screw_axis, false, 0, 0);
}

auto robot = getRobot();
auto dof = robot.numLinks() + 1;
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof);
Vector torque_ID = Vector::Zero(dof);
}  // namespace example

/* ************************************************************************/
// Test forward dynamics with gravity in x direction, zero joint angles
TEST(ForwardDynamics, gravity_x) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, known_torque,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_accelerations =
      example::robot.extractJointAcceleraions(results, example::dof);

  auto expected_accelerations = Vector(4);
  expected_accelerations << -5.88, 5.88, -5.88, 5.88;
  EXPECT(assert_equal(expected_accelerations, actual_accelerations, 1e-6));
}

/* ************************************************************************/
// Test forward dynamics without gravity, torque at first joint
TEST(ForwardDynamics, torque1) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
  known_torque << 10, 0, 0, 0;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, known_torque,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_accelerations =
      example::robot.extractJointAcceleraions(results, example::dof);

  auto expected_accelerations = Vector(4);
  expected_accelerations << 1.5, -1.5, 1.5, -1.5;
  EXPECT(assert_equal(expected_accelerations, actual_accelerations, 1e-6));
}

/* ************************************************************************/
// Test inverse dynamics without gravity, torque at first joint
TEST(InverseDynamics, torque1) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         knowAccelerations = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
  knowAccelerations << 1.5, -1.5, 1.5, -1.5;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, knowAccelerations,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_torques = example::robot.extractTorques(results, example::dof);

  auto expected_torques = Vector(4);
  expected_torques << 10, 0, 0, 0;
  EXPECT(assert_equal(expected_torques, actual_torques, 1e-6));
}

/* ************************************************************************/
// Test forward dynamics with gravity in x direction, torque at two joints
TEST(ForwardDynamics, torque_plus_gravity) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  known_torque << 10, 0, 10, 0;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, known_torque,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_accelerations =
      example::robot.extractJointAcceleraions(results, example::dof);

  auto expected_accelerations = Vector(4);
  expected_accelerations << -2.88, 2.88, -2.88, 2.88;
  EXPECT(assert_equal(expected_accelerations, actual_accelerations, 1e-6));
}

/* ************************************************************************/
// Test inverse dynamics gravity in x direction, torque at first joint
TEST(InverseDynamics, torque_plus_gravity) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         knowAccelerations = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  knowAccelerations << -2.88, 2.88, -2.88, 2.88;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, knowAccelerations,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_torques = example::robot.extractTorques(results, example::dof);

  auto expected_torques = Vector(4);
  expected_torques << 20, 0, 0, 0;
  EXPECT(assert_equal(expected_torques, actual_torques, 1e-6));
}

/* ************************************************************************/
// Test forward dynamics with gravity in x direction, torque at two joints, with
// initial velocity
TEST(ForwardDynamics, torque_gravity_velocity) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  known_torque << 10, 0, 10, 0;
  initialJointVelocities << 10, -10, 10, -10;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, known_torque,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_accelerations =
      example::robot.extractJointAcceleraions(results, example::dof);

  auto expected_accelerations = Vector(4);
  expected_accelerations << -2.88, 2.88, -2.88, 2.88;
  EXPECT(assert_equal(expected_accelerations, actual_accelerations, 1e-6));
}

/* ************************************************************************/
// Test inverse dynamics with gravity in x direction, torque at two joints, with
// initial velocity
TEST(InverseDynamics, torque_gravity_velocity) {
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         knowAccelerations = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  knowAccelerations << -2.88, 2.88, -2.88, 2.88;
  initialJointVelocities << 10, -10, 10, -10;

  DynamicsFactorGraphInput<Vector> dynamicsInput(
      initialJointAngles, initialJointVelocities, knowAccelerations,
      gtsam::Vector6::Zero(), gtsam::Vector6::Zero());
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      dynamicsInput, gravity);
  VectorValues results = factor_graph.optimize();
  auto actual_torques = example::robot.extractTorques(results, example::dof);

  auto expected_torques = Vector(4);
  expected_torques << 20, 0, 0, 0;
  EXPECT(assert_equal(expected_torques, actual_torques, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
