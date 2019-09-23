/**
 * @file testLinearSimpleLoop.cpp
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
vector<URDF_Link> urdf_rrr = {
    URDF_Link(Pose3(Rot3::Rz(HALF_PI), Point3(-1, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, true, 0, 0),
    URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, false, 0, 0),
    URDF_Link(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, false, 0, 0)};
Pose3 base = Pose3();
Pose3 tool = Pose3(Rot3(), Point3(2, 0, 0));

// get screw_axis for loop closure
auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(1, 0, 0));
auto robot = Arm<URDF_Link>(urdf_rrr, base, tool, screw_axis, false, 0, 0);
auto dof = robot.numLinks() + 1;
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof);
Vector torque_ID = Vector::Zero(dof);
}  // namespace example

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
  auto actual_qAccel = example::robot.extractJointAcceleraions(result, example::dof);
  Vector expected_qAccel = Vector::Zero(example::dof);
  expected_qAccel = example::qAccel_ID;
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 10e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
