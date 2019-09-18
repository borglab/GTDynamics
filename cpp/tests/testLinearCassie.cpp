/**
 * @file testLinearCassie.cpp
 * @brief test Cassie close loop with linear dynamics
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

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const double HALF_PI = M_PI / 2;

/* cassie_agility.urdf
    joint1: left-knee-joint (active joint)
            parent link: left-thigh (considered as base, cannot move)
            child link: left-knee
    joint2: left-shin-joint (passive joint, but can exert resistant torque
            because of spring)
            parent link: left-knee
            child link: left-shin
    joint3: left-tarsus-joint
            parent link: left-shin
            child link: left-tarsus
    joint4: left-heel-spring-joint
            parent link: left-tarsus
            child link: left-heel-spring
    joint5: left-rod-joint
            (passive joint, but can exert resistant torque because of spring)
            parent link: left-heel-spring
            child link: left-achilles-rod
    joint6: loop closure joint
            parent link: left-achilles-rod
            child link: left-thigh
*/
namespace example {
vector<URDF_Link> urdf_cassie = {
    URDF_Link(Pose3(Rot3(), Point3(0.12, 0, 0.0045)), Vector3(0, 0, 1),
        'R', 0.7578, Pose3(Rot3(), Point3(0.023, 0.03207, -0.002181)),
        (Matrix(3, 3) << 0.001376, -0.00039744, -4.085e-05, -0.00039744,
         0.0010335, -5.374e-05, -4.085e-05, -5.374e-05, 0.0021637)
            .finished(), true),
    URDF_Link(Pose3(Rot3(), Point3(0.06068, 0.04741, 0)),
        Vector3(0, 0, 1), 'R', 0.577,
        Pose3(Rot3(), Point3(0.18338, 0.001169, 0.0002123)),
        (Matrix(3, 3) << 0.00035939, -0.00020981, 2.266e-05, -0.00020981,
         0.014728, -1.2e-07, 2.266e-05, -1.2e-07, 0.014707)
            .finished(), true),
    URDF_Link(Pose3(Rot3(), Point3(0.43476, 0.02, 0)), Vector3(0, 0, 1),
        'R', 0.782,
        Pose3(Rot3(), Point3(0.11046, -0.03058, -0.00131)),
        (Matrix(3, 3) << 0.00039238, 0.00023651, -4.987e-05, 0.00023651,
         0.013595, -4.82e-06, -4.987e-05, -4.82e-06, 0.013674)
            .finished(), true),
    URDF_Link(Pose3(Rot3::RzRyRx(0, 0, 2.7207), Point3(-0.01269, -0.03059, 0)),
        Vector3(0, 0, 1), 'R', 0.126,
        Pose3(Rot3(), Point3(0.081, 0.0022, 0)),
        (Matrix(3, 3) << 2.959e-05, 7.15e-06, -6e-07, 7.15e-06, 0.00022231,
         1e-07, -6e-07, 1e-07, 0.0002007)
            .finished(), false),
    URDF_Link(Pose3(Rot3::RzRyRx(0, 0, 0.608212), Point3(0.11877, -0.01, 0)),
        Vector3(0, 0, 1), 'R', 0.157,
        Pose3(Rot3(), Point3(0.2472, 0, 0)),
        Vector3(0.000004, 0.014061, 0.014061).asDiagonal(), false)};

Pose3 base = Pose3();
Pose3 tool = Pose3(Rot3(), Point3(0.418, 0, 0));
auto robot = Arm<URDF_Link>(urdf_cassie, base, tool);
auto dof = robot.numLinks();
// get screw_axis for loop closure
auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(0.000202169, 3.86353e-05, 0.0045));
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof);
Vector torque_ID = Vector::Zero(dof);
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
  known_q << -1.201826, 0, 1.428819, 0, -1.481429, 1.254439;
  auto factor_graph = example::robot.closedLoopInverseDynamicsFactorGraph(
      known_q, known_qVel, known_qAccel, example::screw_axis, false, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qTorque = example::robot.extractTorques(result, N);
  Vector expected_qTorque = Vector::Zero(example::dof + 1);
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
  known_q << -1.201826, 0, 1.428819, 0, -1.481429, 1.254439;
  known_torque = example::torque_ID;
  auto factor_graph = example::robot.closedLoopForwardDynamicsFactorGraph(
      known_q, known_qVel, known_torque, example::screw_axis, base_twist_accel,
      external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  int N = example::dof+1;
  auto actual_qAccel = example::robot.extractJointAcceleraions(result, N);
  Vector expected_qAccel = Vector::Zero(example::dof + 1);
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 10e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}