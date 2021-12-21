/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsInterval.cpp
 * @brief Test Kinematics methods defined on intervals.
 * @author: Frank Dellaert, Yetong Zhang
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

#include <algorithm>

#include "contactGoalsExample.h"

TEST(Interval, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  // TODO(frank): the goals for contact will differ for a Interval vs Slice.
  using namespace contact_goals_example;

  // Create an interval.
  const size_t num_time_steps = 5;
  const Interval interval(0, num_time_steps - 1);

  // Instantiate kinematics algorithms
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  auto graph = kinematics.graph(interval, robot);
  EXPECT_LONGS_EQUAL(12 * num_time_steps, graph.size());

  auto objectives = kinematics.pointGoalObjectives(interval, contact_goals);
  EXPECT_LONGS_EQUAL(4 * num_time_steps, objectives.size());

  auto objectives2 = kinematics.jointAngleObjectives(interval, robot);
  EXPECT_LONGS_EQUAL(12 * num_time_steps, objectives2.size());

  auto result = kinematics.inverse(interval, robot, contact_goals);

  // Check that goals are achieved
  constexpr double tol = 1e-5;
  for (const ContactGoal& goal : contact_goals) {
    for (size_t k = 0; k < num_time_steps; k++) {
      EXPECT(goal.satisfied(result, k, tol));
    }
  }
}

TEST(Interval, Interpolate) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a second contact goal to transition to, moving RF foot by 10 cm.
  auto contact_goals2 = contact_goals;
  contact_goals2[2] = {{RF, contact_in_com}, {0.4, -0.16, -0.2}};

  // Create expected values for start and end times
  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::SOFT_CONSTRAINTS;
  Kinematics kinematics(parameters);
  auto result1 = kinematics.inverse(Slice(5), robot, contact_goals);
  auto result2 = kinematics.inverse(Slice(9), robot, contact_goals);

  // Create a kinematic trajectory over timesteps 5, 6, 7, 8, 9 that
  // interpolates between goal configurations at timesteps 5 and 9.
  gtsam::Values result =
      kinematics.interpolate(Interval(5, 9), robot, contact_goals, contact_goals2);
  EXPECT(result.exists(internal::PoseKey(0, 5)));
  EXPECT(result.exists(internal::PoseKey(0, 9)));
  EXPECT(assert_equal(Pose(result1, 0, 5), Pose(result, 0, 5)));
  EXPECT(assert_equal(Pose(result2, 0, 9), Pose(result, 0, 9)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
