/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsInterval.cpp
 * @brief Test moving for a trajectory consisting of multiple phases.
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

/**
 * Interpolate using inverse kinematics: the goals are linearly interpolated.
 * All results are return in values.
 */
gtsam::Values Kinematics::interpolate(
    const Interval& interval, const ContactGoals& contact_goals1,
    const ContactGoals& contact_goals2) const {
  gtsam::Values result;
  const double dt = 1.0 / (interval.k_start - interval.k_end);  // 5 6 7 8 9 [10
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    const double t = dt * (k - interval.k_start);
    ContactGoals goals;
    transform(contact_goals1.begin(), contact_goals1.end(),
              contact_goals2.begin(), std::back_inserter(goals),
              [t](const ContactGoal& goal1, const ContactGoal& goal2) {
                return ContactGoal{
                    goal1.point_on_link,
                    (1.0 - t) * goal1.goal_point + t * goal2.goal_point};
              });
    result.insert(inverse(Slice(k), goals));
  }
  return result;
}

TEST(Interval, Interpolate) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a second contact goal to transition to, moving RF foot by 10 cm.
  auto contact_goals2 = contact_goals;
  contact_goals2[2] = {{RF, contact_in_com}, {0.4, -0.16, -0.2}};

  // Create expected values for start and end times
  Kinematics kinematics(robot);
  auto result1 = kinematics.inverse(Slice(5), contact_goals);
  auto result2 = kinematics.inverse(Slice(9), contact_goals);

  // Create a kinematic trajectory that interpolates between two configurations.
  gtsam::Values result =
      kinematics.interpolate(Interval(5, 9), contact_goals, contact_goals2);
  EXPECT(result.exists(internal::PoseKey(0, 5)));
  EXPECT(result.exists(internal::PoseKey(0, 9)));
  EXPECT(assert_equal(Pose(result1, 0, 5), Pose(result, 0, 5)));
  EXPECT(assert_equal(Pose(result2, 0, 9), Pose(result, 0, 9)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
