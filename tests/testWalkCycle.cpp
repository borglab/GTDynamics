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

namespace {
class GeneralContactSpec : public ConstraintSpec {
 public:
  explicit GeneralContactSpec(PointOnLinks points) : points_(std::move(points)) {}
  const PointOnLinks& contactPoints() const override { return points_; }
  void print(const std::string&) const override {}

 private:
  PointOnLinks points_;
};
}  // namespace

TEST(WalkCycle, GeneralSpecificationsAndFlight) {
  using namespace walk_cycle_example;
  const PointOnLink a(links_1[0], Point3(0, 0, 0.1));
  const PointOnLink b(links_1[1], Point3(0, 0, 0.3), false);
  const PointOnLink c(links_1[2], Point3(0, 0, 0.2));
  const auto first = std::make_shared<GeneralContactSpec>(PointOnLinks{a, b});
  const auto second = std::make_shared<GeneralContactSpec>(PointOnLinks{b, c});
  const auto flight = std::make_shared<GeneralContactSpec>(PointOnLinks{});
  const WalkCycle cycle(std::vector<Phase>{
      Phase(3, 4, first), Phase(4, 5, second), Phase(6, 7, flight),
      Phase(8, 9, second)});

  EXPECT_LONGS_EQUAL(3, cycle.contactPoints().size());
  EXPECT_LONGS_EQUAL(2, cycle.getPhaseContactPoints(0).size());
  EXPECT(!cycle.getPhaseContactPoints(0)[1].isGround);
  const auto all = cycle.allPhasesContactPoints();
  EXPECT_LONGS_EQUAL(4, all.size());
  EXPECT_LONGS_EQUAL(0, all[2].size());
  const auto transitions = cycle.transitionContactPoints();
  EXPECT_LONGS_EQUAL(3, transitions.size());
  EXPECT_LONGS_EQUAL(1, transitions[0].size());
  EXPECT(transitions[0][0].equals(b));
  EXPECT_LONGS_EQUAL(0, transitions[1].size());
  EXPECT_LONGS_EQUAL(0, transitions[2].size());
  EXPECT(cycle.getPhaseSwingLinks(0) == std::vector<std::string>{c.link->name()});
  EXPECT_LONGS_EQUAL(3, cycle.getPhaseSwingLinks(2).size());

  constexpr double ground_height = 7.0;
  const auto goals = cycle.initContactPointGoal(robot, ground_height);
  EXPECT_LONGS_EQUAL(3, goals.size());
  DOUBLES_EQUAL(ground_height, goals.at(a.link->name()).z(), 1e-12);
  DOUBLES_EQUAL(ground_height, goals.at(c.link->name()).z(), 1e-12);
  EXPECT(gtsam::assert_equal(b.link->bMcom() * b.point,
                             goals.at(b.link->name()), 1e-12));

  const Trajectory trajectory(cycle, 1);
  EXPECT_LONGS_EQUAL(4, trajectory.phaseContactPoints().size());
  EXPECT(trajectory.transitionContactPoints()[0][0].equals(b));
  EXPECT(WalkCycle().transitionContactPoints().empty());
}

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

  auto phase0 = std::make_shared<FootContactConstraintSpec>(phase_0_links,
                                                              contact_in_com);
  auto phase1 = std::make_shared<FootContactConstraintSpec>(phase_1_links,
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

  // Use a non-zero ground height to exercise explicit wiring.
  constexpr double ground_height = 4.2;
  Point3 goal_LH(-0.371306, 0.1575, ground_height);   // LH
  Point3 goal_LF(0.278694, 0.1575, ground_height);    // LF
  Point3 goal_RF(0.278694, -0.1575, ground_height);   // RF
  Point3 goal_RH(-0.371306, -0.1575, ground_height);  // RH

  // Check initalization of contact goals.
  auto cp_goals = walk_cycle.initContactPointGoal(robot, ground_height);
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
