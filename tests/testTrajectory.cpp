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
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/utils/Trajectory.h"
#include "gtdynamics/utils/WalkCycle.h"

using namespace gtsam;
using namespace std;

auto kModel1 = gtsam::noiseModel::Unit::Create(1);
auto kModel6 = gtsam::noiseModel::Unit::Create(6);

using gtdynamics::ContactPoints;
using gtdynamics::Phase;
using gtdynamics::Robot;

// Class to test protected method
class TrajectoryTest : public gtdynamics::Trajectory {
 public:
  TrajectoryTest() : Trajectory(){};

  const ContactPoints getIntersection(ContactPoints CPs_1,
                                      ContactPoints CPs_2) const {
    return Trajectory::getIntersection(CPs_1, CPs_2);
  }
};

TEST(Trajectory, Intersection) {
  Robot robot =
      gtdynamics::CreateRobotFromFile(SDF_PATH + "/test/spider.sdf", "spider");

  double contact_height = 5;
  size_t num_time_steps = 1;

  // Initialize first phase
  auto phase_1 = Phase(robot, num_time_steps);
  phase_1.addContactPoint("tarsus_2", {3., 3., 3.}, contact_height);
  phase_1.addContactPoint("tarsus_1", {3., 3., 3.}, contact_height);
  phase_1.addContactPoint("tarsus_3", {3., 3., 3.}, contact_height);

  // Initialize second phase
  auto phase_2 = Phase(robot, num_time_steps);
  phase_2.addContactPoint("tarsus_3", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_4", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_5", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_2", {3., 3., 3.}, contact_height);

  TrajectoryTest traj;
  ContactPoints intersection = traj.getIntersection(
      phase_1.getAllContactPoints(), phase_2.getAllContactPoints());

  ContactPoints expected = {{"tarsus_2", {{3., 3., 3.}, 0, contact_height}},
                            {"tarsus_3", {{3., 3., 3.}, 0, contact_height}}};

  for (auto const& contact_point : intersection) {
    EXPECT(expected[contact_point.first] == contact_point.second);
  }
}

TEST(Trajectory, error) {
  Robot robot =
      gtdynamics::CreateRobotFromFile(SDF_PATH + "/test/spider.sdf", "spider");

  // Initialize first phase
  size_t num_time_steps = 2;
  auto phase_1 = Phase(robot, num_time_steps);
  double contact_height = 5;
  phase_1.addContactPoint("tarsus_1", {3., 3., 3.}, contact_height);
  phase_1.addContactPoint("tarsus_2", {3., 3., 3.}, contact_height);
  phase_1.addContactPoint("tarsus_3", {3., 3., 3.}, contact_height);

  // Initialize second phase
  size_t num_time_steps_2 = 3;
  auto phase_2 = Phase(robot, num_time_steps_2);
  phase_2.addContactPoint("tarsus_2", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_3", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_4", {3., 3., 3.}, contact_height);
  phase_2.addContactPoint("tarsus_5", {3., 3., 3.}, contact_height);

  // Initialize walk cycle
  auto walk_cycle = gtdynamics::WalkCycle();
  walk_cycle.addPhase(phase_1);
  walk_cycle.addPhase(phase_2);

  // Initialize Trajectory
  size_t repeat = 5;
  auto trajectory = gtdynamics::Trajectory(walk_cycle, repeat);

  auto phase_cps = trajectory.phaseContactPoints();
  EXPECT_LONGS_EQUAL(10, phase_cps.size());
  EXPECT_LONGS_EQUAL(3, phase_cps[2].size());

  auto trans_cps = trajectory.transitionContactPoints();
  EXPECT_LONGS_EQUAL(9, trans_cps.size());
  EXPECT_LONGS_EQUAL(2, trans_cps[1].size());

  auto phase_durations = trajectory.phaseDurations();
  EXPECT_LONGS_EQUAL(2, phase_durations[2]);

  auto robot_models = trajectory.phaseRobotModels();
  EXPECT_LONGS_EQUAL(10, robot_models.size());

  auto final_timesteps = trajectory.finalTimeSteps();
  EXPECT_LONGS_EQUAL(7, final_timesteps[2]);
  EXPECT_LONGS_EQUAL(6, trajectory.getStartTimeStep(2));
  EXPECT_LONGS_EQUAL(7, trajectory.getEndTimeStep(2));
  EXPECT_LONGS_EQUAL(5, trajectory.getLinks().size());
  EXPECT_LONGS_EQUAL(4, trajectory.getPhaseContactLinks(3).size());
  EXPECT_LONGS_EQUAL(1, trajectory.getPhaseSwingLinks(3).size());

  auto prev_cp = trajectory.initContactPointGoal();
  EXPECT_LONGS_EQUAL(5, prev_cp.size());

  double gaussian_noise = 1e-5;
  vector<Values> transition_graph_init =
      trajectory.transitionPhaseInitialValues(gaussian_noise);
  EXPECT_LONGS_EQUAL(9, transition_graph_init.size());

  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;
  double sigma_dynamics = 1e-5;  // std of dynamics constraints.
  auto opt = gtdynamics::OptimizerSetting(sigma_dynamics);
  auto graph_builder = gtdynamics::DynamicsGraph(opt, gravity);
  vector<gtsam::NonlinearFactorGraph> transition_graphs =
      trajectory.getTransitionGraphs(graph_builder, mu);
  // 5 repeates, 2 phaes -> 10 phases, 9 transitions
  EXPECT_LONGS_EQUAL(9, transition_graphs.size());
  // regression test
  EXPECT_LONGS_EQUAL(205, transition_graphs[0].size());

  // Test multi-phase factor graph.
  auto multi_phase_graph = trajectory.multiPhaseFactorGraph(
      graph_builder, gtdynamics::DynamicsGraph::CollocationScheme::Euler, mu);
  // regression test
  EXPECT_LONGS_EQUAL(6760, multi_phase_graph.size());

  // Test objectives for contact links.
  const double ground_height = 0.0;
  auto contact_link_objectives = trajectory.contactLinkObjectives(
      noiseModel::Isotropic::Sigma(3, 1e-7), ground_height);
  // regression test
  EXPECT_LONGS_EQUAL(130, contact_link_objectives.size());

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
