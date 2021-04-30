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
  const gtsam::Vector3 gravity(0, 0, -10);
  constexpr double sigma_dynamics = 1e-5;
  StaticsParameters parameters(sigma_dynamics, gravity);
  Statics statics(robot, parameters);

  // Get an inverse kinematics solution
  auto ik_solution = statics.Kinematics::inverse(slice, contact_goals);

  // Test graph generation
  auto graph = statics.graph(slice);
  EXPECT_LONGS_EQUAL(37, graph.size());
  // GTD_PRINT(graph);

  // Test initialization
  auto values = statics.initialValues(slice);
  EXPECT_LONGS_EQUAL(36, values.size());

  // Solve for wrenches, with known kinematics
  auto result = statics.solve(slice, ik_solution);
  EXPECT_LONGS_EQUAL(61, result.size());
  GTD_PRINT(result);
  EXPECT_DOUBLES_EQUAL(0, Torque(result, 0, k), 1e-5);

  // Optimize kinematics while minimizing torque
  auto minimal = statics.minimizeTorques(slice);
  EXPECT_LONGS_EQUAL(61, minimal.size());
  // GTD_PRINT(minimal);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
