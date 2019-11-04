/**
 * @file  testReducedHybridDynamics.cpp
 * @brief test hybrid dynamics factor graph with base and tool factor reduced
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <DhLink.h>
#include <UrdfLink.h>

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
vector<DhLink> dh_rr = {DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3),
                        DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3)};

// Create Puma robot.
auto robot = Arm<DhLink>(dh_rr, Pose3(), Pose3());
auto dof = robot.numLinks();

Vector joint_angles = Vector::Zero(dof);
Vector joint_velocities = Vector::Zero(dof);
Vector torques = Vector::Zero(dof);
Vector joint_accelerations = Vector::Zero(dof);
Vector base_twist_accel = Vector::Zero(6);
Vector external_wrench = Vector::Zero(6);
Vector expected_joint_accelerations = Vector::Zero(dof);
Vector expected_torques = Vector::Zero(dof);
}  // namespace example

TEST(Arm, reducedHybridDynamics_1) {
  Vector3 gravity(0, -9.8, 0);
  example::external_wrench << 0, 0, 0, 0, -2.5, 0;
  example::joint_accelerations << 5, -20;
  example::torques << 0, 0;
  example::expected_joint_accelerations << 5, -20;
  example::expected_torques << 0, 0;

  map<size_t, double> joint_accels, joint_torques;
  joint_accels.emplace(0, example::joint_accelerations[0]);
  joint_torques.emplace(1, example::torques[1]);
  Arm<DhLink>::AngularVariablesPair givenVariablesPair(joint_accels,
                                                       joint_torques);
  DynamicsFactorGraphInput<Arm<DhLink>::AngularVariablesPair>
      hybridDynamicsInput(example::joint_angles, example::joint_velocities,
                          givenVariablesPair, example::base_twist_accel,
                          example::external_wrench);
  auto result = example::robot.reducedHybridDynamics(hybridDynamicsInput);

  map<size_t, double> expected_joint_accels, expected_joint_torques;
  expected_joint_accels.emplace(1, example::expected_joint_accelerations[1]);
  expected_joint_torques.emplace(0, example::expected_torques[0]);
  EXPECT(result.first == expected_joint_accels);
  EXPECT(result.second == expected_joint_torques);
}

TEST(Arm, reducedHybridDynamics_2) {
  Vector3 gravity(0, -9.8, 0);
  example::external_wrench << 0, 0, 0, 0, 0, 0;
  example::joint_accelerations << -9.8, 19.6;
  example::torques << 0, 0;
  example::expected_joint_accelerations << -9.8, 19.6;
  example::expected_torques << 0, 0;

  map<size_t, double> joint_accels, joint_torques;
  joint_accels.emplace(0, example::joint_accelerations[0]);
  joint_torques.emplace(1, example::torques[1]);

  Arm<DhLink>::AngularVariablesPair givenVariablesPair(joint_accels,
                                                       joint_torques);
  DynamicsFactorGraphInput<Arm<DhLink>::AngularVariablesPair>
      hybridDynamicsInput(example::joint_angles, example::joint_velocities,
                          givenVariablesPair, example::base_twist_accel,
                          example::external_wrench);
  auto result =
      example::robot.reducedHybridDynamics(hybridDynamicsInput, gravity);

  map<size_t, double> expected_joint_accels, expected_joint_torques;
  expected_joint_accels.emplace(1, example::expected_joint_accelerations[1]);
  expected_joint_torques.emplace(0, example::expected_torques[0]);
  EXPECT(assert_equal(result.second[0], expected_joint_torques[0], 1e-5));
  EXPECT(assert_equal(result.first[1], expected_joint_accels[1], 1e-5));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
