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
#include <gtdynamics/statics/Statics.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

TEST(Phase, Statics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a slice.
  const size_t k = 1;
  const Slice slice(k);

  // Instantiate statics algorithms
  StaticsParameters parameters;
  Statics statics(robot, parameters);

  // Get an inverse kinematics solution
  auto ik_solution = statics.Kinematics::inverse(slice, contact_goals);

  // Now, solve for wrenches
  auto result = statics.solve(slice, ik_solution);
  GTD_PRINT(result);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
