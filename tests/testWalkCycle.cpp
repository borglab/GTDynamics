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
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/WalkCycle.h>

#include "walkCycleExample.h"

using namespace gtdynamics;
using gtsam::Point3;

// Class to test protected method
class WalkCycleTest : public WalkCycle {
 public:
  WalkCycleTest() : WalkCycle(){};
  using WalkCycle::getIntersection;
};

TEST(WalkCycle, Intersection) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("spider.sdf"), "spider");

  using namespace walk_cycle_example;
  WalkCycleTest wc;
  PointOnLinks intersection =
      wc.getIntersection(phase_1->contactPoints(), phase_2->contactPoints());

  PointOnLinks expected = {{robot.link("tarsus_2_L2"), contact_in_com},
                           {robot.link("tarsus_3_L3"), contact_in_com}};

  for (size_t i = 0; i < 2; i++) {
    EXPECT(gtsam::assert_equal(expected[i], intersection[i]));
  }
}

TEST(WalkCycle, contactPoints) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("spider.sdf"), "spider");

  using namespace walk_cycle_example;
  auto walk_cycle_phases = walk_cycle.phases();
  EXPECT_LONGS_EQUAL(3, walk_cycle.getPhaseContactPoints(0).size());
  EXPECT_LONGS_EQUAL(4, walk_cycle.getPhaseContactPoints(1).size());
  EXPECT_LONGS_EQUAL(2, walk_cycle.numPhases());
  EXPECT_LONGS_EQUAL(num_time_steps + num_time_steps_2,
                     walk_cycle.numTimeSteps());
  EXPECT_LONGS_EQUAL(5, walk_cycle.contactPoints().size());
}

TEST(WalkCycle, objectives) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("vision60.urdf"), "spider");
  EXPECT_LONGS_EQUAL(13, robot.numLinks());

  constexpr size_t num_time_steps = 5;
  const Point3 contact_in_com(0.14, 0, 0);

  std::vector<LinkSharedPtr> phase_0_links = {robot.link("lower1"),
                                              robot.link("lower2")};
  std::vector<LinkSharedPtr> phase_1_links = {robot.link("lower0"),
                                              robot.link("lower3")};

  auto phase0 = boost::make_shared<FootContactConstraintSpec>(phase_0_links,
                                                              contact_in_com);
  auto phase1 = boost::make_shared<FootContactConstraintSpec>(phase_1_links,
                                                              contact_in_com);

  FootContactVector states = {phase0, phase1};
  std::vector<size_t> phase_lengths = {num_time_steps, num_time_steps};

  auto walk_cycle =
      WalkCycle({phase0, phase1}, {num_time_steps, num_time_steps});

  // check Phase swing links function
  auto swing_links0 = walk_cycle.getPhaseSwingLinks(0);
  auto swing_links1 = walk_cycle.getPhaseSwingLinks(1);
  EXPECT_LONGS_EQUAL(swing_links0.size(), 2);
  EXPECT_LONGS_EQUAL(swing_links1.size(), 2);

  // Expected contact goal points.
  Point3 goal_LH(-0.371306, 0.1575, 0);   // LH
  Point3 goal_LF(0.278694, 0.1575, 0);    // LF
  Point3 goal_RF(0.278694, -0.1575, 0);   // RF
  Point3 goal_RH(-0.371306, -0.1575, 0);  // RH

  // Check initalization of contact goals.
  auto cp_goals = walk_cycle.initContactPointGoal(robot, -0.191839);
  EXPECT_LONGS_EQUAL(4, cp_goals.size());
  EXPECT(gtsam::assert_equal(goal_LH, cp_goals["lower1"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_LF, cp_goals["lower0"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_RF, cp_goals["lower2"], 1e-6));
  EXPECT(gtsam::assert_equal(goal_RH, cp_goals["lower3"], 1e-6));

  const Point3 step(0, 0.4, 0);
  const gtsam::SharedNoiseModel cost_model = gtsam::noiseModel::Unit::Create(3);

  // Check creation of PointGoalFactors.
  gtsam::NonlinearFactorGraph factors =
      walk_cycle.contactPointObjectives(step, cost_model, 0, &cp_goals);
  EXPECT_LONGS_EQUAL(num_time_steps * 2 * 4, factors.size());

  // Check goals have been updated
  EXPECT(gtsam::assert_equal<Point3>(goal_LH + step, cp_goals["lower1"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_LF + step, cp_goals["lower0"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_RF + step, cp_goals["lower2"], 1e-6));
  EXPECT(gtsam::assert_equal<Point3>(goal_RH + step, cp_goals["lower3"], 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
