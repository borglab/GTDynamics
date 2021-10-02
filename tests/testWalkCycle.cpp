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
#include "gtdynamics/utils/Trajectory.h"
#include "walkCycleExample.h"

using namespace gtdynamics;
using gtsam::Point3;

TEST(WalkCycle, contactPoints) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");

  using namespace walk_cycle_example;
  Trajectory trajectory(walk_cycle,1);
  auto walk_cycle_phases = walk_cycle.phases();
  EXPECT_LONGS_EQUAL(3, walk_cycle_phases[0].footContactConstraintSpec()->contactPoints().size());
  EXPECT_LONGS_EQUAL(4, walk_cycle_phases[1].footContactConstraintSpec()->contactPoints().size());
  EXPECT_LONGS_EQUAL(2, walk_cycle.numPhases());
  EXPECT_LONGS_EQUAL(num_time_steps + num_time_steps_2,
                     walk_cycle.numTimeSteps());
  EXPECT_LONGS_EQUAL(5, trajectory.contactPoints().size());
}

TEST(WalkCycle, objectives) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");
  EXPECT_LONGS_EQUAL(13, robot.numLinks());

  constexpr size_t num_time_steps = 5;
  const Point3 contact_in_com(0.14, 0, 0);

  std::vector<LinkSharedPtr> phase_0_links = {robot.link("lower1"), robot.link("lower2")};
  std::vector<LinkSharedPtr> phase_1_links = {robot.link("lower0"), robot.link("lower3")};

  auto phase0 = boost::make_shared<FootContactConstraintSpec>(phase_0_links, contact_in_com);
  auto phase1 = boost::make_shared<FootContactConstraintSpec>(phase_1_links, contact_in_com);
  
  FootContactVector states = {phase0, phase1};
  std::vector<size_t> phase_lengths = {num_time_steps, num_time_steps};

  auto walk_cycle = WalkCycle({phase0, phase1}, {num_time_steps, num_time_steps});
  Trajectory trajectory(walk_cycle,1);

  // Expected contact goal points.
  Point3 goal_LH(-0.371306, 0.1575, 0);   // LH
  Point3 goal_LF(0.278694, 0.1575, 0);    // LF
  Point3 goal_RF(0.278694, -0.1575, 0);   // RF
  Point3 goal_RH(-0.371306, -0.1575, 0);  // RH

  // Check initalization of contact goals.
  auto cp_goals = trajectory.initContactPointGoal(robot, -0.191839);
  EXPECT_LONGS_EQUAL(4, cp_goals.size());
  EXPECT(gtsam::assert_equal(goal_LH, cp_goals["lower1"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_LF, cp_goals["lower0"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_RF, cp_goals["lower2"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_RH, cp_goals["lower3"], 1e-6));

  const Point3 step(0, 0.4, 0);
  const gtsam::SharedNoiseModel cost_model = nullptr;

  // Check creation of PointGoalFactors.
  ContactPointGoals updated_cp_goals;
  gtsam::NonlinearFactorGraph factors =
      trajectory.contactPointObjectives(robot, cost_model, step, updated_cp_goals, -0.191839);
  EXPECT_LONGS_EQUAL(num_time_steps * 2 * 4, factors.size());

  // Check goals have been updated
  EXPECT(gtsam::assert_equal<Point3>(goal_LH + step, updated_cp_goals["lower1"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_LF + step, updated_cp_goals["lower0"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_RF + step, updated_cp_goals["lower2"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_RH + step, updated_cp_goals["lower3"], 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
