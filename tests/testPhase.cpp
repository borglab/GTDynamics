/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPhase.cpp
 * @brief Test Phase class.
 * @author: Frank Dellaert, Tarushree Gandhi, Disha Das
 */

#include <CppUnitLite/TestHarness.h>

#include <sstream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/Phase.h"
#include "walkCycleExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;

TEST(Phase, All) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");

  using namespace walk_cycle_example;
  ContactPoint cp = phase_1.contactPoint("tarsus_3_L3");
  EXPECT(assert_equal(contact_in_com, cp.point));

  ContactPoints cps = phase_1.contactPoints();
  EXPECT_LONGS_EQUAL(3, cps.size());
  EXPECT(assert_equal(contact_in_com, cps["tarsus_1_L1"].point));
  EXPECT_LONGS_EQUAL(2, phase_1.numTimeSteps());

  // Check printing
  std::stringstream ss;
  ss << phase_1;
  EXPECT(
      "[tarsus_1_L1: {[   0 0.19    0], 0}, tarsus_2_L2: {[   0 0.19    0], 0},"
      " tarsus_3_L3: {[   0 0.19    0], 0}, ]" == ss.str());

  // Test hasContact.
  EXPECT(phase_1.hasContact("tarsus_1_L1"));
  EXPECT(phase_1.hasContact("tarsus_2_L2"));
  EXPECT(phase_1.hasContact("tarsus_3_L3"));
  EXPECT(!phase_1.hasContact("tarsus_4_L4"));
  EXPECT(!phase_1.hasContact("tarsus_5_R4"));

  // contactPointObjectives
  const Point3 step(0, 0.4, 0);
  const gtsam::SharedNoiseModel cost_model = nullptr;
  Point3 goal(1, 2, 3);
  const size_t k_start = 777;
  Phase::ContactPointGoals cp_goals = {{"tarsus_2_L2", goal},
                                       {"tarsus_1_L1", goal},
                                       {"tarsus_3_L3", goal},
                                       {"tarsus_4_L4", goal},
                                       {"tarsus_5_R4", goal}};
  gtsam::NonlinearFactorGraph factors = phase_1.contactPointObjectives(
      walk_cycle.contactPoints(), step, cost_model, robot, k_start, cp_goals);
  auto new_goals = phase_1.updateContactPointGoals(walk_cycle.contactPoints(),
                                                   step, cp_goals);
  EXPECT_LONGS_EQUAL(num_time_steps * 5, factors.size());
  EXPECT(assert_equal(goal, new_goals["tarsus_2_L2"]));
  EXPECT(assert_equal(goal, new_goals["tarsus_1_L1"]));
  EXPECT(assert_equal(goal, new_goals["tarsus_3_L3"]));
  EXPECT(assert_equal(Point3(goal + step), new_goals["tarsus_4_L4"]));
  EXPECT(assert_equal(Point3(goal + step), new_goals["tarsus_5_R4"]));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
