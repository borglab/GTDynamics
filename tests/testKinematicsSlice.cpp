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
#include <gtdynamics/utils/Slice.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

TEST(Slice, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a slice.
  const size_t k = 777;
  const Slice slice(k);

  // Instantiate kinematics algorithms
  auto parameters = boost::make_shared<KinematicsParameters>();
  parameters->method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  // Create initial values
  auto values = kinematics.initialValues(slice, robot, 0.0);
  EXPECT_LONGS_EQUAL(13 + 12, values.size());

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
    EXPECT(goal.satisfied(fk, k, 0.05));
  }

  auto graph = kinematics.graph(slice, robot);
  EXPECT_LONGS_EQUAL(12, graph.size());

  auto objectives = kinematics.pointGoalObjectives(slice, contact_goals);
  EXPECT_LONGS_EQUAL(4, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(slice, robot);
  EXPECT_LONGS_EQUAL(12, objectives2.size());

  auto result = kinematics.inverse(slice, robot, contact_goals);

  // Check that well-determined
  graph.add(objectives);
  graph.add(objectives2);
  EXPECT_LONGS_EQUAL(12 + 12 + 4, graph.size());
  // auto factor = graph.linearizeToHessianFactor(result);
  // GTD_PRINT(*factor);

  // Check that goals are achieved
  constexpr double tol = 1e-5;
  for (const ContactGoal& goal : contact_goals) {
    EXPECT(goal.satisfied(result, k, tol));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
