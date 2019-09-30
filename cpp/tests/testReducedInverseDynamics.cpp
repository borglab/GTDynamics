/**
 * @file  testReducedInverseDynamics.cpp
 * @brief test inverse dynamics factor graph with base and tool factor reduced
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <DHLink.h>
#include <URDFLink.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/Factor.h>

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
vector<DH_Link> dh_rr = {
    DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3),
    DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3)};

// Create Puma robot.
auto robot = Arm<DH_Link>(dh_rr, Pose3(), Pose3());
auto dof = robot.numLinks();
Vector joint_angles = Vector::Zero(dof);
Vector joint_velocities = Vector::Zero(dof);
Vector joint_accelerations = Vector::Zero(dof);
Vector base_twist_accel = Vector::Zero(6);
Vector external_wrench = Vector::Zero(6);
Vector expected_torques = Vector::Zero(dof);
}  // namespace example

/* ======== gravity compensation case: assume Y-axis is up ========= */
TEST(Arm, reducedInverseDynamics_1) {
  example::joint_accelerations << -9.8, 19.6;
  example::base_twist_accel << 0, 0, 0, 0, 0, 0;
  Vector3 gravity(0, -9.8, 0);
  example::external_wrench << 0, 0, 0, 0, 0, 0;
  example::expected_torques << 0, 0;
  DynamicsFactorGraphInput<Vector> inverseDynamicsInput(
      example::joint_angles, example::joint_velocities,
      example::joint_accelerations, example::base_twist_accel,
      example::external_wrench);
  GaussianFactorGraph factor_graph =
      example::robot.reducedInverseDynamicsFactorGraph(
          inverseDynamicsInput, gravity);

  VectorValues result = factor_graph.optimize();
  auto actual_torques = example::robot.extractTorques(result);
  EXPECT(assert_equal(example::expected_torques, actual_torques));
}

/* ========= test case when an external wrench is applied ========== */
TEST(Arm, reducedInverseDynamics_2) {
  example::joint_accelerations << 5, -20;
  example::base_twist_accel << 0, 0, 0, 0, 0, 0;
  example::external_wrench << 0, 0, 0, 0, -2.5, 0;
  example::expected_torques << 0, 0;
  DynamicsFactorGraphInput<Vector> inverseDynamicsInput(
      example::joint_angles, example::joint_velocities,
      example::joint_accelerations, example::base_twist_accel,
      example::external_wrench);
  GaussianFactorGraph factor_graph =
      example::robot.reducedInverseDynamicsFactorGraph(
          inverseDynamicsInput);
 
  VectorValues result = factor_graph.optimize();
  auto actual_torques = example::robot.extractTorques(result);
  EXPECT(assert_equal(example::expected_torques, actual_torques));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
