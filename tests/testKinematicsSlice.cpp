/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsSlice.cpp
 * @brief Test Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Slice.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

TEST(Phase, inverse_kinematics) {
  Robot robot = CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"));

  // Create a slice.
  const size_t k = 777;
  const Slice slice(k);

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  // parameters.lm_parameters.setVerbosityLM("SUMMARY");
  parameters.lm_parameters.setlambdaInitial(1e7);
  parameters.lm_parameters.setAbsoluteErrorTol(1e-3);
  Kinematics<Slice> kinematics(robot, parameters);

  // Create initial values
  auto values = kinematics.initialValues(slice, 0.0);
  EXPECT_LONGS_EQUAL(13 + 12, values.size());

  // establish contact/goal pairs
  const Point3 contact_in_com(0.14, 0, 0);
  const ContactGoals contact_goals = {
      {{robot.link("lower1"), contact_in_com}, {-0.4, 0.16, -0.2}},    // LH
      {{robot.link("lower0"), contact_in_com}, {0.3, 0.16, -0.2}},     // LF
      {{robot.link("lower2"), contact_in_com}, {0.3, -0.16, -0.2}},    // RF
      {{robot.link("lower3"), contact_in_com}, {-0.4, -0.16, -0.2}}};  // RH

  // Set twists to zero for FK. TODO(frank): separate kinematics from velocity?
  for (auto&& link : robot.links()) {
    InsertTwist(&values, link->id(), k, gtsam::Z_6x1);
  }

  // Do forward kinematics
  const std::string root_link_name("body");
  const auto root_link_id = robot.link(root_link_name)->id();
  EXPECT(values.exists(internal::PoseKey(root_link_id, k)));
  auto fk = robot.forwardKinematics(values, k, std::string(root_link_name));

  // Check goals with FK solution
  for (const ContactGoal& goal : contact_goals) {
    // EXPECT(assert_equal(goal.goal_point, goal.predict(fk, k)));
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  auto graph = kinematics.graph(slice);
  EXPECT_LONGS_EQUAL(12, graph.size());

  auto objectives = kinematics.pointGoalObjectives(slice, contact_goals);
  EXPECT_LONGS_EQUAL(4, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(slice);
  EXPECT_LONGS_EQUAL(12, objectives2.size());

  // TODO(frank): consider renaming ContactPoint to PointOnLink
  auto result = kinematics.inverse(slice, contact_goals);

  // Check that well-determined
  graph.add(objectives);
  graph.add(objectives2);
  EXPECT_LONGS_EQUAL(12 + 12 + 4, graph.size());
  // auto factor = graph.linearizeToHessianFactor(result);
  // GTD_PRINT(*factor);

  // Check that goals are achieved
  constexpr double tol = 0.01;
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(result, k, tol));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
