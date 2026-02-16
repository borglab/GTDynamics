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

namespace {

ContactGoals InterpolateContactGoals(const ContactGoals& contact_goals_start,
                                     const ContactGoals& contact_goals_end,
                                     double alpha) {
  ContactGoals contact_goals;
  std::transform(
      contact_goals_start.begin(), contact_goals_start.end(),
      contact_goals_end.begin(), std::back_inserter(contact_goals),
      [alpha](const ContactGoal& start_goal, const ContactGoal& end_goal) {
        return ContactGoal{
            start_goal.point_on_link,
            (1.0 - alpha) * start_goal.goal_point + alpha * end_goal.goal_point};
      });
  return contact_goals;
}

gtsam::Values ManualInterpolateByInverse(
    const Kinematics& kinematics, const Interval& interval, const Robot& robot,
    const ContactGoals& contact_goals_start,
    const ContactGoals& contact_goals_end) {
  gtsam::Values result;
  const double denominator =
      static_cast<double>(interval.k_end - interval.k_start);
  for (size_t k = interval.k_start; k <= interval.k_end; ++k) {
    const double alpha =
        denominator > 0.0
            ? static_cast<double>(k - interval.k_start) / denominator
            : 0.0;
    const auto goals =
        InterpolateContactGoals(contact_goals_start, contact_goals_end, alpha);
    result.insert(kinematics.inverse(Slice(k), robot, goals));
  }
  return result;
}

}  // namespace

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
  constexpr double tol = 1e-4;
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
  auto result2 = kinematics.inverse(Slice(9), robot, contact_goals2);

  // Create a kinematic trajectory over timesteps 5, 6, 7, 8, 9 that
  // interpolates between goal configurations at timesteps 5 and 9.
  gtsam::Values result = kinematics.interpolate(Interval(5, 9), robot,
                                                contact_goals, contact_goals2);
  EXPECT(result.exists(PoseKey(0, 5)));
  EXPECT(result.exists(PoseKey(0, 9)));
  EXPECT(assert_equal(Pose(result1, 0, 5), Pose(result, 0, 5)));
  EXPECT(assert_equal(Pose(result2, 0, 9), Pose(result, 0, 9)));
}

TEST(Interval, InterpolateMatchesManualInverseLoop) {
  using namespace contact_goals_example;

  // Move RF foot by 10 cm in +x at the end of interval.
  auto contact_goals2 = contact_goals;
  contact_goals2[2] = {{RF, contact_in_com}, {0.4, -0.16, -0.2}};

  KinematicsParameters parameters;
  parameters.method = OptimizationParameters::Method::AUGMENTED_LAGRANGIAN;
  Kinematics kinematics(parameters);

  const Interval interval(5, 9);
  const auto manual_result = ManualInterpolateByInverse(
      kinematics, interval, robot, contact_goals, contact_goals2);
  const auto interpolate_result =
      kinematics.interpolate(interval, robot, contact_goals, contact_goals2);

  // Interpolate should match an explicit per-k call to inverse on interpolated
  // goal points.
  constexpr double tol = 1e-5;
  for (size_t k = interval.k_start; k <= interval.k_end; ++k) {
    for (const auto& joint : robot.joints()) {
      const auto key = JointAngleKey(joint->id(), k);
      EXPECT(manual_result.exists(key));
      EXPECT(interpolate_result.exists(key));
      EXPECT_DOUBLES_EQUAL(manual_result.at<double>(key),
                           interpolate_result.at<double>(key), tol);
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
