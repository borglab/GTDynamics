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
  parameters.lm_parameters.setVerbosityLM("SUMMARY");
  parameters.lm_parameters.setlambdaInitial(1e7);
  parameters.lm_parameters.setAbsoluteErrorTol(1e-3);
  Kinematics<Phase> kinematics(robot, phase, parameters);

  auto graph = kinematics.graph();
  EXPECT_LONGS_EQUAL(12 * num_time_steps, graph.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
