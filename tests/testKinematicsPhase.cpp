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

#include "contactGoalsExample.h"

TEST(Phase, InverseKinematics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  constexpr size_t num_time_steps = 5;
  Phase phase0(num_time_steps, {LH,RF}, contact_in_com);
  // TODO(frank): test methods producing constraints.
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
