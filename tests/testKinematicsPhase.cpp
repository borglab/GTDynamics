/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsPhase.cpp
 * @brief Test Kinematics for a phase with fixed contacts.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Phase.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

TEST(Phase, inverse_kinematics) {
  Robot robot = CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"));

  // Create a phase.
  const size_t num_time_steps = 5;
  const Phase phase(num_time_steps);

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  //   parameters.lm_parameters.setVerbosityLM("SUMMARY");
  parameters.lm_parameters.setlambdaInitial(1e7);
  parameters.lm_parameters.setAbsoluteErrorTol(1e-3);
  Kinematics<Phase> kinematics(robot, parameters);

  auto graph = kinematics.graph(phase);
  EXPECT_LONGS_EQUAL(12 * num_time_steps, graph.size());

  // establish contact/goal pairs
  // TODO(frank): the goals for contact will differ for a Phase vs Slice.
  const Point3 contact_in_com(0.14, 0, 0);
  const ContactGoals contact_goals = {
      {{robot.link("lower1"), contact_in_com}, {-0.4, 0.16, -0.2}},    // LH
      {{robot.link("lower0"), contact_in_com}, {0.3, 0.16, -0.2}},     // LF
      {{robot.link("lower2"), contact_in_com}, {0.3, -0.16, -0.2}},    // RF
      {{robot.link("lower3"), contact_in_com}, {-0.4, -0.16, -0.2}}};  // RH

  auto objectives = kinematics.pointGoalObjectives(phase, contact_goals);
  EXPECT_LONGS_EQUAL(4 * num_time_steps, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(phase);
  EXPECT_LONGS_EQUAL(12 * num_time_steps, objectives2.size());

  // TODO(frank): consider renaming ContactPoint to PointOnLink
  auto result = kinematics.inverse(phase, contact_goals);

  // Check that goals are achieved
  constexpr double tol = 0.01;
  for (const ContactGoal& goal : contact_goals) {
    for (size_t k = 0; k < num_time_steps; k++) {
      EXPECT(assert_equal(goal.goal_point, goal.predict(result, k), tol));
    }
  }

  // Check that goals are achieved
  for (const ContactGoal& goal : contact_goals) {
    for (size_t k = 0; k < num_time_steps; k++) {
      EXPECT(goal.satisfied(result, k, tol));
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
