/**
 * @file  testLinearHybridDynamics.cpp
 * @brief test linear hybrid dynamics
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <DhLink.h>
#include <UrdfLink.h>
#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/timing.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
vector<DhLink> dh_puma = {
    DhLink(0, 0.0000, 0.0000, +90, 'R', 0, Point3(0, 0, 0),
           Vector3(0, 0.35, 0).asDiagonal()),
    DhLink(0, 0.4318, 0, 0.0, 'R', 17.40, Point3(-0.3638, 0.006, 0.2275),
           Vector3(0.130, 0.524, 0.539).asDiagonal()),
    DhLink(0, 0.0203, 0.15005, -90, 'R', 4.80, Point3(-0.0203, -0.0141, 0.0700),
           Vector3(0.066, 0.086, 0.0125).asDiagonal()),
    DhLink(0, 0, 0.4318, +90, 'R', 0.82, Point3(0, 0.19, 0),
           Vector3(0.0018, 0.0013, 0.0018).asDiagonal()),
    DhLink(0, 0.0000, 0.0000, -90, 'R', 0.34, Point3(0, 0, 0),
           Vector3(0.0003, 0.0004, 0.0003).asDiagonal()),
    DhLink(0, 0.0000, 0.0000, 0.0, 'R', 0.09, Point3(0, 0, 0.032),
           Vector3(0.00015, 0.00015, 0.00004).asDiagonal())};

// Create Puma robot.
auto robot = Arm<DhLink>(dh_puma, Pose3(), Pose3());
auto dof = robot.numLinks();

Vector joint_angles = Vector::Zero(dof);
Vector joint_velocities = Vector::Zero(dof);
Vector joint_accelerations = Vector::Zero(dof);
Vector torques = Vector::Zero(dof);
Vector base_twist_accel = Vector::Zero(6);
Vector external_wrench = Vector::Zero(6);
Vector3 gravity(0, 0, -9.8);
}  // namespace example

TEST(dynamics, hybrid) {
  example::joint_velocities << -5, -10, -15, -20, -25, -30;
  example::torques << 2.14242560e+00, -4.72874900e+01, 1.37677604e+01,
      2.15162000e-01, 1.45261716e-03, 7.67944871e-05;
  example::joint_accelerations << 0.174533, 0.349066, 0.523599, 0.698132,
      0.872665, 1.047198;
  /* ====================== test hybrid dynamics ===================== */
  map<size_t, double> joint_accels, joint_torques;
  // for puma
  joint_accels.emplace(0, example::joint_accelerations[0]);
  joint_accels.emplace(1, example::joint_accelerations[1]);
  joint_accels.emplace(2, example::joint_accelerations[2]);
  joint_torques.emplace(3, example::torques[3]);
  joint_torques.emplace(4, example::torques[4]);
  joint_torques.emplace(5, example::torques[5]);

  Arm<DhLink>::AngularVariablesPair givenVariablesPair(joint_accels,
                                                       joint_torques);
  DynamicsFactorGraphInput<Arm<DhLink>::AngularVariablesPair>
      hybridDynamicsInput(example::joint_angles,
                          radians(example::joint_velocities),
                          givenVariablesPair, example::base_twist_accel,
                          example::external_wrench);

  auto hybridResult =
      example::robot.hybridDynamics(hybridDynamicsInput, example::gravity);

  map<size_t, double> expected_joint_accels, expected_joint_torques;
  expected_joint_accels.emplace(3, example::joint_accelerations[3]);
  expected_joint_accels.emplace(4, example::joint_accelerations[4]);
  expected_joint_accels.emplace(5, example::joint_accelerations[5]);
  expected_joint_torques.emplace(0, example::torques[0]);
  expected_joint_torques.emplace(1, example::torques[1]);
  expected_joint_torques.emplace(2, example::torques[2]);
  EXPECT(assert_equal(hybridResult.second[0], expected_joint_torques[0], 1e-5));
  EXPECT(assert_equal(hybridResult.second[1], expected_joint_torques[1], 1e-5));
  EXPECT(assert_equal(hybridResult.second[2], expected_joint_torques[2], 1e-5));
  EXPECT(assert_equal(hybridResult.first[3], expected_joint_accels[3], 1e-5));
  EXPECT(assert_equal(hybridResult.first[4], expected_joint_accels[4], 1e-5));
  EXPECT(assert_equal(hybridResult.first[5], expected_joint_accels[5], 1e-5));
  /* ================================================================= */
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
