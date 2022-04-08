/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTrajectory.cpp
 * @brief Test Trajectory class.
 * @author: Frank Dellaert, Tarushree Gandhi, Disha Das, Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/utils/Trajectory.h"
#include "gtdynamics/utils/WalkCycle.h"
#include "walkCycleExample.h"

using namespace gtsam;
using namespace std;

auto kModel1 = gtsam::noiseModel::Unit::Create(1);
auto kModel6 = gtsam::noiseModel::Unit::Create(6);

using namespace gtdynamics;

TEST(Trajectory, error) {
  using namespace walk_cycle_example;
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("spider.sdf"), "spider");

  // Initialize Trajectory
  size_t repeat = 3;
  using namespace walk_cycle_example;
  auto trajectory = Trajectory(walk_cycle, repeat);

  // test phase method
  EXPECT_LONGS_EQUAL(2, trajectory.phase(0).numTimeSteps());
  EXPECT_LONGS_EQUAL(3, trajectory.phase(1).numTimeSteps());
  EXPECT_LONGS_EQUAL(2, trajectory.phase(2).numTimeSteps());
  EXPECT_LONGS_EQUAL(3, trajectory.phase(3).numTimeSteps());

  auto phase_cps = trajectory.phaseContactPoints();
  EXPECT_LONGS_EQUAL(repeat * 2, phase_cps.size());
  EXPECT_LONGS_EQUAL(3, phase_cps[2].size());

  auto trans_cps = trajectory.transitionContactPoints();
  EXPECT_LONGS_EQUAL(5, trans_cps.size());
  for (auto cps : trans_cps) {
    EXPECT_LONGS_EQUAL(2, cps.size());
  }

  auto phase_durations = trajectory.phaseDurations();
  EXPECT_LONGS_EQUAL(2, phase_durations[2]);

  auto final_timesteps = trajectory.finalTimeSteps();
  EXPECT_LONGS_EQUAL(7, final_timesteps[2]);
  EXPECT_LONGS_EQUAL(6, trajectory.getStartTimeStep(2));
  EXPECT_LONGS_EQUAL(7, trajectory.getEndTimeStep(2));

  auto cp_goals = walk_cycle.initContactPointGoal(robot, 0);
  EXPECT_LONGS_EQUAL(5, cp_goals.size());
  //regression
  EXPECT(gtsam::assert_equal(gtsam::Point3(-0.926417, 1.19512, 0.000151302),
                             cp_goals["tarsus_2_L2"], 1e-5));

  double gaussian_noise = 1e-5;
  Initializer initializer;
  vector<Values> transition_graph_init =
      trajectory.transitionPhaseInitialValues(robot, initializer, gaussian_noise);
  EXPECT_LONGS_EQUAL(5, transition_graph_init.size());

  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;
  double sigma_dynamics = 1e-5;  // std deviation for dynamics constraints.
  auto opt = OptimizerSetting(sigma_dynamics);
  auto graph_builder = DynamicsGraph(opt, gravity);

  // Check transition graphs
  vector<gtsam::NonlinearFactorGraph> transition_graphs =
      trajectory.getTransitionGraphs(robot, graph_builder, mu);
  EXPECT_LONGS_EQUAL(repeat * 2 - 1, transition_graphs.size());
  // regression test
  EXPECT_LONGS_EQUAL(203, transition_graphs[0].size());

  // Test multi-phase factor graph.
  auto graph = trajectory.multiPhaseFactorGraph(robot, graph_builder,
                                                CollocationScheme::Euler, mu);
  // regression test
  EXPECT_LONGS_EQUAL(4298, graph.size());
  EXPECT_LONGS_EQUAL(4712, graph.keys().size());

  Values init_vals = trajectory.multiPhaseInitialValues(robot, initializer, 1e-5, 1. / 240);
  EXPECT_LONGS_EQUAL(4712, init_vals.size());

  // Test objectives for contact links.
  const Point3 step(0, 0.4, 0);
  auto contact_link_objectives = trajectory.contactPointObjectives(
      robot, noiseModel::Isotropic::Sigma(3, 1e-7), step, 0);
  // steps = 2+3 per walk cycle, 5 legs involved
  const size_t expected = repeat * ((2 + 3) * 5);
  EXPECT_LONGS_EQUAL(expected, contact_link_objectives.size());
  // // regression
  // auto last_factor = boost::dynamic_pointer_cast<PointGoalFactor>(
  //     contact_link_objectives.back());
  // EXPECT(gtsam::assert_equal(gtsam::Point3(-0.190001, -0.300151, 0.000151302),
  //                            last_factor->goalPoint(), 1e-5));

  // Test boundary conditions.
  NonlinearFactorGraph boundary_conditions;
  trajectory.addBoundaryConditions(&boundary_conditions, robot, kModel6,
                                   kModel6, kModel6, kModel1, kModel1);
  // regression test
  EXPECT_LONGS_EQUAL(260, boundary_conditions.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
