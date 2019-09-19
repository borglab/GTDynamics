/**
 * @file testLinearClosedLoop.cpp
 * @brief test robotic manipulator with closed kinematic loop with linear factor
 * graph
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <URDFLink.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const double HALF_PI = M_PI / 2;

namespace example {
vector<URDF_Link> urdf_5r = {
    URDF_Link(Pose3(Rot3::Rz(HALF_PI), Point3(-0.2, 0, 0)), Vector3(0, 0, 1), 'R',
              2, Pose3(Rot3(), Point3(0.4, 0, 0)), Z_3x3, true),
    URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(0.8, 0, 0)), Vector3(0, 0, 1), 'R',
              2.25, Pose3(Rot3(), Point3(0.45, 0, 0)), Z_3x3, false),
    URDF_Link(Pose3(Rot3::Rz(-M_PI/3), Point3(0.4, 0, 0)), Vector3(0, 0, 1), 'R',
              sqrt(3), Pose3(Rot3(), Point3(0.2*sqrt(3), 0, 0)), Z_3x3, false),
    URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(0.4*sqrt(3), 0, 0)), Vector3(0, 0, 1), 'R',
              1, Pose3(Rot3(), Point3(0.2, 0, 0)), Z_3x3, false)};
Pose3 base = Pose3();
Pose3 tool = Pose3(Rot3(), Point3(0.4, 0, 0));
// get screw_axis for loop closure
auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(0.2, 0, 0));
auto robot = Arm<URDF_Link>(urdf_5r, base, tool, screw_axis, true);
auto dof = robot.numLinks();
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof + 1);
Vector torque_ID = Vector::Zero(dof + 1);
}  // namespace example

/* ************************************************************************* */
/**
 * Test inverse dynamics with gravity in y direction
 */
TEST(ID_factor_graph, gravity_y) {
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();
  Vector known_q = Vector::Zero(example::dof + 1),
         known_qVel = Vector::Zero(example::dof + 1),
         known_qAccel = Vector::Zero(example::dof + 1),
         known_torque = Vector::Zero(example::dof + 1),
         base_twist_accel = Vector::Zero(6),
         external_wrench = Vector::Zero(6);
  known_qVel << 0.5, -0.5, 0.5, 0, -0.5;
  known_qVel << 0.15, -0.15, 0.15, 0, -0.15;
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      known_q, known_qVel, known_qAccel, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qTorque = example::robot.extractTorques(result, N);
  Vector expected_qTorque = Vector::Zero(example::dof + 1);
  expected_qTorque << -15.3492717, -7.49532381e-17, 2.37320893e-17, 1.52357895e-16, -30.8287271;
  EXPECT(assert_equal(expected_qTorque, actual_qTorque, 10e-6));
  example::torque_ID = actual_qTorque;
  example::qAccel_ID = known_qAccel;
}

/* ************************************************************************* */
/**
 * Test forward dynamics with gravity in y direction
 */
TEST(FD_factor_graph, gravity_y) {
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();
  Vector known_q = Vector::Zero(example::dof + 1),
         known_qVel = Vector::Zero(example::dof + 1),
         known_qAccel = Vector::Zero(example::dof + 1),
         known_torque = Vector::Zero(example::dof + 1),
         base_twist_accel = Vector::Zero(6),
         external_wrench = Vector::Zero(6);
  known_qVel << 0.5, -0.5, 0.5, 0, -0.5;
  known_qVel << 0.15, -0.15, 0.15, 0, -0.15;
  known_torque = example::torque_ID;
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      known_q, known_qVel, known_torque, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qAccel = example::robot.extractJointAcceleraions(result, N);
  Vector expected_qAccel = Vector::Zero(example::dof + 1);
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 10e-6));
}

/* ************************************************************************* */
/**
 * Test inverse dynamics with gravity in x direction
 */
TEST(ID_factor_graph, gravity_x) {
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  Vector known_q = Vector::Zero(example::dof + 1),
         known_qVel = Vector::Zero(example::dof + 1),
         known_qAccel = Vector::Zero(example::dof + 1),
         known_torque = Vector::Zero(example::dof + 1),
         base_twist_accel = Vector::Zero(6),
         external_wrench = Vector::Zero(6);
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      known_q, known_qVel, known_qAccel, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qTorque = example::robot.extractTorques(result, N);
  Vector expected_qTorque = Vector::Zero(example::dof + 1);
  expected_qTorque << 32.2696392, 6.30340537e-32, 2.77081645e-16, 0, -4.50611469;
  EXPECT(assert_equal(expected_qTorque, actual_qTorque, 10e-6));
  example::torque_ID = actual_qTorque;
  example::qAccel_ID = known_qAccel;
}

/* ************************************************************************* */
/**
 * Test forward dynamics with gravity in x direction
 */
TEST(FD_factor_graph, gravity_x) {
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  Vector known_q = Vector::Zero(example::dof + 1),
         known_qVel = Vector::Zero(example::dof + 1),
         known_qAccel = Vector::Zero(example::dof + 1),
         known_torque = Vector::Zero(example::dof + 1),
         base_twist_accel = Vector::Zero(6),
         external_wrench = Vector::Zero(6);
  known_torque = example::torque_ID;
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      known_q, known_qVel, known_torque, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qAccel = example::robot.extractJointAcceleraions(result, N);
  Vector expected_qAccel = Vector::Zero(example::dof + 1);
  expected_qAccel = example::qAccel_ID;
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 10e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
