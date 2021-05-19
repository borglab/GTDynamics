/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWalkCycle.cpp
 * @brief Test WalkCycle class.
 * @author: Frank Dellaert, Tarushree Gandhi, Disha Das
 */

#include <CppUnitLite/TestHarness.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/utils/WalkCycle.h"
#include "walkCycleExample.h"

using namespace gtdynamics;
using gtsam::Point3;

TEST(WalkCycle, error) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");

  using namespace walk_cycle_example;
  auto walk_cycle_phases = walk_cycle.phases();
  EXPECT_LONGS_EQUAL(3, walk_cycle_phases[0].contactPoints().size());
  EXPECT_LONGS_EQUAL(4, walk_cycle_phases[1].contactPoints().size());
  EXPECT_LONGS_EQUAL(2, walk_cycle.numPhases());
  EXPECT_LONGS_EQUAL(num_time_steps + num_time_steps_2,
                     walk_cycle.numTimeSteps());
  EXPECT_LONGS_EQUAL(5, walk_cycle.contactPoints().size());
}

TEST(WalkCycle, robotModel) {

  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");

  // First phase
  constexpr size_t num_time_steps = 2;
  gtsam::Point3 contact_in_com(0, 0.19, 0);
  Phase phase_1(robot, num_time_steps);
  phase_1.addContactPoint("tarsus_1_L1", contact_in_com);
  phase_1.addContactPoint("tarsus_2_L2", contact_in_com);

  // Second phase
  auto robot_2 = gtdynamics::CreateRobotFromFile(
      kSdfPath + std::string("/test/simple_rr.sdf"), "simple_rr_sdf");
  Phase phase_2(robot_2, num_time_steps);
  phase_2.addContactPoint("link_0", contact_in_com);
  phase_2.addContactPoint("link_1", contact_in_com);

  try {
    auto walk_cycle = WalkCycle({phase_1, phase_2});
  } catch (const std::runtime_error &error) {
    EXPECT(error.what() ==
           std::string("This phase belongs to a different Robot model!"));
  }
}

TEST(Phase, inverse_kinematics) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");
  EXPECT_LONGS_EQUAL(13, robot.numLinks());

  constexpr size_t num_time_steps = 5;
  const Point3 contact_in_com(0.14, 0, 0);
  Phase phase0(robot, num_time_steps), phase1(robot, num_time_steps);
  phase0.addContactPoint("lower1", contact_in_com);  // LH
  phase1.addContactPoint("lower0", contact_in_com);  // LF
  phase0.addContactPoint("lower2", contact_in_com);  // RF
  phase1.addContactPoint("lower3", contact_in_com);  // RH
  auto walk_cycle = WalkCycle({phase0, phase1});

  // Set goal points to reasonable values
  Point3 goal_LH(0, 0.15, 0);     // LH
  Point3 goal_LF(0.6, 0.15, 0);   // LF
  Point3 goal_RF(0.6, -0.15, 0);  // RF
  Point3 goal_RH(0, -0.15, 0);    // RH

  const Point3 step(0, 0.4, 0);
  const gtsam::SharedNoiseModel cost_model = nullptr;
  const size_t k = 777;
  ContactGoals cp_goals = {
      ContactGoal(PointOnLink(robot.link("lower1"), contact_in_com), goal_LH),
      ContactGoal(PointOnLink(robot.link("lower0"), contact_in_com), goal_LF),
      ContactGoal(PointOnLink(robot.link("lower2"), contact_in_com), goal_RF),
      ContactGoal(PointOnLink(robot.link("lower3"), contact_in_com), goal_RH)};
  gtsam::NonlinearFactorGraph factors =
      walk_cycle.contactPointObjectives(step, cost_model, k, &cp_goals);
  EXPECT_LONGS_EQUAL(num_time_steps * 2 * 4, factors.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
