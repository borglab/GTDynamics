/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWalkCycle.cpp
 * @brief Test WalkCycle class.
 * @Author: Frank Dellaert, Tarushree Gandhi, Disha Das
 */

#include <CppUnitLite/TestHarness.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/utils/WalkCycle.h"

using namespace gtdynamics;

TEST(WalkCycle, error) {
  Robot robot_configuration =
      CreateRobotFromFile(kSdfPath + std::string("/test/spider.sdf"), "spider");

  // Initialize first phase
  size_t num_time_steps = 20;
  auto phase_1 = gtdynamics::Phase(robot_configuration, num_time_steps);
  double contact_height = 5;
  phase_1.addContactPoint("tarsus_1", gtsam::Point3(3, 3, 3), contact_height);
  phase_1.addContactPoint("tarsus_2", gtsam::Point3(3, 3, 3), contact_height);
  phase_1.addContactPoint("tarsus_3", gtsam::Point3(3, 3, 3), contact_height);

  // Initialize second phase
  size_t num_time_steps_2 = 25;
  auto phase_2 = gtdynamics::Phase(robot_configuration, num_time_steps_2);
  phase_2.addContactPoint("tarsus_2", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_3", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_4", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_5", gtsam::Point3(3, 3, 3), contact_height);

  // Initialize walk cycle
  auto walk_cycle = gtdynamics::WalkCycle();
  walk_cycle.addPhase(phase_1);
  walk_cycle.addPhase(phase_2);

  auto walk_cycle_phases = walk_cycle.phases();
  EXPECT(walk_cycle_phases[0].contactPoints().size() == 3);
  EXPECT(walk_cycle_phases[1].contactPoints().size() == 4);
  EXPECT(walk_cycle.numPhases() == 2);
  EXPECT(walk_cycle.allContactPoints().size() == 5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
