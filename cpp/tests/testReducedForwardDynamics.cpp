/**
 * @file  testReducedForwardDynamics.cpp
 * @brief test forward dynamics factor graph with base and tool factor reduced
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <DHLink.h>
#include <URDFLink.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
vector<DH_Link> dh_rr = {DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3),
                         DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3)};

// Create Puma robot.
auto robot = Arm<DH_Link>(dh_rr, Pose3(), Pose3());
auto dof = robot.numLinks();

Vector joint_angles = Vector::Zero(dof);
Vector joint_velocities = Vector::Zero(dof);
Vector torques = Vector::Zero(dof);
Vector base_twist_accel = Vector::Zero(6);
Vector external_wrench = Vector::Zero(6);
Vector expected_joint_accelerations = Vector::Zero(dof);
}  // namespace example

/* ======== gravity compensation case: assume Y-axis is up ========= */
TEST(Arm, reducedForwardDynamics_1) {
  Vector3 gravity(0, -9.8, 0);
  example::external_wrench << 0, 0, 0, 0, 0, 0;
  example::expected_joint_accelerations << -9.8, 19.6;

  GaussianFactorGraph factor_graph =
      example::robot.reducedForwardDynamicsFactorGraph(
          example::joint_angles, example::joint_velocities, example::torques,
          example::base_twist_accel, example::external_wrench, gravity);

  VectorValues result = factor_graph.optimize();
  auto actual_acceleration = example::robot.extractJointAcceleraions(result);
  EXPECT(assert_equal(example::expected_joint_accelerations, actual_acceleration));
}

/* ========= test case when an external wrench is applied ========== */
TEST(Arm, reducedForwardDynamics_2) {
  example::external_wrench << 0, 0, 0, 0, -2.5, 0;
  example::expected_joint_accelerations << 5, -20;

  GaussianFactorGraph factor_graph =
      example::robot.reducedForwardDynamicsFactorGraph(
          example::joint_angles, example::joint_velocities,
          example::torques, example::base_twist_accel,
          example::external_wrench);

  VectorValues result = factor_graph.optimize();
  auto actual_acceleration = example::robot.extractJointAcceleraions(result);
  EXPECT(assert_equal(example::expected_joint_accelerations, actual_acceleration));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
