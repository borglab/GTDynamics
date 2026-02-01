/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsConstraints.cpp
 * @brief Isolate evaluation of kinematics constraints.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>

#include <cmath>

#include "contactGoalsExample.h"

using namespace gtdynamics;

namespace {

bool AllViolationsFinite(const gtsam::NonlinearEqualityConstraints& constraints,
                         const gtsam::Values& values) {
  for (size_t i = 0; i < constraints.size(); ++i) {
    gtsam::NonlinearEqualityConstraints single;
    single.push_back(constraints.at(i));
    const double violation = single.violationNorm(values);
    if (!std::isfinite(violation)) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(KinematicsConstraints, SliceViolation) {
  using namespace contact_goals_example;

  const size_t k = 777;
  const Slice slice(k);

  KinematicsParameters parameters;
  Kinematics kinematics(parameters);

  auto constraints = kinematics.constraints(slice, robot);
  auto goal_constraints = kinematics.pointGoalConstraints(slice, contact_goals);
  for (const auto& constraint : goal_constraints) {
    constraints.push_back(constraint);
  }

  auto values = kinematics.initialValues(slice, robot, 0.0);

  EXPECT(AllViolationsFinite(constraints, values));
}

TEST(KinematicsConstraints, IntervalViolation) {
  using namespace contact_goals_example;

  const size_t num_time_steps = 5;
  const Interval interval(0, num_time_steps - 1);

  KinematicsParameters parameters;
  Kinematics kinematics(parameters);

  auto constraints = kinematics.constraints(interval, robot);
  auto goal_constraints =
      kinematics.pointGoalConstraints(interval, contact_goals);
  for (const auto& constraint : goal_constraints) {
    constraints.push_back(constraint);
  }

  auto values = kinematics.initialValues(interval, robot, 0.0);

  EXPECT(AllViolationsFinite(constraints, values));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
