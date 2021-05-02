/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testStaticsSlice.cpp
 * @brief Test Statics in single time slice.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "contactGoalsExample.h"
#include "gtdynamics/statics/Statics.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"

using namespace gtdynamics;
using namespace gtsam;
using std::map;
using std::string;

constexpr double kTol = 1e-6;
constexpr double kSigmaDynamics = 1e-5;

// Simple, base+link example from Colab notebook
// https://colab.research.google.com/drive/17QfCocrG49EVJ15vNg5viWtOXIB48Q5M
TEST(Statics, OneMovingLink) {
  constexpr double g = 10, mass = 1;  // for nice round numbers.
  const Vector3 gravity2D(0, -g, 0);
  constexpr double theta = M_PI / 3;
  const Rot3 wRcom = Rot3::Rz(theta);
  constexpr double L = 2;  // meters
  const Pose3 wTcom(wRcom, wRcom.rotate(Point3(L / 2, 0, 0)));

  // Check that gravity wrench is correct when expressed in CoM.
  const Vector6 Fg_B = GravityWrench(gravity2D, mass, wTcom);
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, -8.66025404, -5, 0).finished(),
                      Fg_B, kTol));

  // Create base and link
  // TODO(frank): #207 should not have to provide wTl to Link constructor
  const Pose3 wTl;  // we don't care!
  const Pose3 lTcom(Rot3(), Point3(L / 2, 0, 0));
  const auto I3 = Matrix3::Identity();  // inertia
  auto base =
      boost::make_shared<Link>(0, "base", 1e10, I3, Pose3(), Pose3(), true);
  auto link = boost::make_shared<Link>(1, "link", 1.0, I3, wTl, lTcom);

  // Create joint
  constexpr unsigned char id = 22;
  // TODO(frank): #206 should not have to provide wTj to the joint constructor.
  const Pose3 wTj;
  // TODO(frank): #205 make JointParams last argument and provide default
  const JointParams jointParams;
  const Vector3 axis(0, 0, 1);
  auto joint = boost::make_shared<RevoluteJoint>(id, "joint1", wTj, base, link,
                                                 jointParams, axis);

  // Create mechanism.
  // TODO(frank): specifying name is redundant and failure prone!
  const Robot robot({{"base", base}, {"link", link}}, {{"joint1", joint}});
  // TODO(frank): We need to be able to create a robot without having to do this
  base->addJoint(joint);
  link->addJoint(joint);

  // Assert correctness of torque by hand, as in Colab.
  // The screw_axis converts from COM to joint frame and dots with Z-axis.
  Vector6 screw_axis = joint->screwAxis(link);
  const double tau = screw_axis.dot(-Fg_B);
  constexpr double expected_tau = 5;
  EXPECT_DOUBLES_EQUAL(expected_tau, tau, kTol);

  // Now do statics using GTD.
  StaticsParameters parameters2D(kSigmaDynamics, gravity2D);
  // parameters2D.lm_parameters.setVerbosityLM("SUMMARY");
  Statics statics(robot, parameters2D);
  const size_t k = 777;
  const Slice slice(k);
  Values values;
  InsertJointAngle(&values, joint->id(), k, theta);

  // Do forward kinematics and check this agrees with example.
  Values fk_solution = robot.forwardKinematics(values, k);
  EXPECT(assert_equal(wTcom, Pose(fk_solution, link->id(), k), kTol));

  // Now solve for wrenches/torques.
  auto result = statics.solve(slice, fk_solution);

  // Check that the actual wrench on the link is the negative gravity wrench.
  const Vector6 actualWrench =
      Wrench(result, joint->child()->id(), joint->id(), k);
  Vector expectedWrench = -Fg_B;
  EXPECT(assert_equal(expectedWrench, actualWrench, kTol));

  // Check the resulting torque, 5 Nm in counterclockwise, in two different
  // ways: once from the wrench, and once from the optimization:
  EXPECT_DOUBLES_EQUAL(
      5, joint->transformWrenchToTorque(joint->child(), actualWrench), kTol);
  EXPECT_DOUBLES_EQUAL(expected_tau, Torque(result, joint->id(), k), 1e-5);
}

// Do test with Quadruped and desired contact goals.
TEST(Statics, Quadruped) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Instantiate statics algorithms
  const Vector3 kGravity(0, 0, -10);
  StaticsParameters parameters3D(kSigmaDynamics, kGravity);
  Statics statics(robot, parameters3D);

  // Get an inverse kinematics solution
  const size_t k = 1;
  const Slice slice(k);
  auto ik_solution = statics.Kinematics::inverse(slice, contact_goals);

  // Test graph generation
  auto graph = statics.graph(slice);
  EXPECT_LONGS_EQUAL(37, graph.size());

  // Test initialization
  auto values = statics.initialValues(slice);
  EXPECT_LONGS_EQUAL(36, values.size());

  // Solve for wrenches, with known kinematics
  auto result = statics.solve(slice, ik_solution);
  EXPECT_LONGS_EQUAL(61, result.size());
  // Regression
  EXPECT_DOUBLES_EQUAL(0.1518, Torque(result, 0, k), 1e-5);

  // Optimize kinematics while minimizing torque
  auto minimal = statics.minimizeTorques(slice);
  EXPECT_LONGS_EQUAL(61, minimal.size());
  // GTD_PRINT(minimal);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

