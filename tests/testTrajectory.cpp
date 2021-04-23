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
  Robot robot = gtdynamics::CreateRobotFromFile(
      gtdynamics::kSdfPath + std::string("/spider.sdf"), "spider");

  size_t num_time_steps = 1;

  // Initialize first phase
  auto phase_1 = Phase(robot, num_time_steps);
  phase_1.addContactPoint("tarsus_2_L2", {3., 3., 3.});
  phase_1.addContactPoint("tarsus_1_L1", {3., 3., 3.});
  phase_1.addContactPoint("tarsus_3_L3", {3., 3., 3.});

  // Initialize second phase
  auto phase_2 = Phase(robot, num_time_steps);
  phase_2.addContactPoint("tarsus_3_L3", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_4_L4", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_5_R4", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_2_L2", {3., 3., 3.});

  TrajectoryTest traj;
  ContactPoints intersection =
      traj.getIntersection(phase_1.contactPoints(), phase_2.contactPoints());

  ContactPoints expected = {{"tarsus_2_L2", {{3., 3., 3.}, 0}},
                            {"tarsus_3_L3", {{3., 3., 3.}, 0}}};

  for (auto const& contact_point : intersection) {
    EXPECT(expected[contact_point.first] == contact_point.second);
  }
}

TEST(Trajectory, error) {
  Robot robot = gtdynamics::CreateRobotFromFile(
      gtdynamics::kSdfPath + std::string("/spider.sdf"), "spider");

  // Initialize first phase
  size_t num_time_steps = 2;
  auto phase_1 = Phase(robot, num_time_steps);
  phase_1.addContactPoint("tarsus_1_L1", {3., 3., 3.});
  phase_1.addContactPoint("tarsus_2_L2", {3., 3., 3.});
  phase_1.addContactPoint("tarsus_3_L3", {3., 3., 3.});

  // Initialize second phase
  size_t num_time_steps_2 = 3;
  auto phase_2 = Phase(robot, num_time_steps_2);
  phase_2.addContactPoint("tarsus_2_L2", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_3_L3", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_4_L4", {3., 3., 3.});
  phase_2.addContactPoint("tarsus_5_R4", {3., 3., 3.});

  // Initialize walk cycle
  auto walk_cycle = gtdynamics::WalkCycle();
  walk_cycle.addPhase(phase_1);
  walk_cycle.addPhase(phase_2);

  // Initialize Trajectory
  size_t repeat = 3;
  auto trajectory = gtdynamics::Trajectory(walk_cycle, repeat);

  auto phase_cps = trajectory.phaseContactPoints();
  EXPECT_LONGS_EQUAL(repeat * 2, phase_cps.size());
  EXPECT_LONGS_EQUAL(3, phase_cps[2].size());

  auto trans_cps = trajectory.transitionContactPoints();
  EXPECT_LONGS_EQUAL(5, trans_cps.size());
  EXPECT_LONGS_EQUAL(2, trans_cps[1].size());

  auto phase_durations = trajectory.phaseDurations();
  EXPECT_LONGS_EQUAL(2, phase_durations[2]);

  auto robot_models = trajectory.phaseRobotModels();
  EXPECT_LONGS_EQUAL(6, robot_models.size());

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
  EXPECT_LONGS_EQUAL(5, transition_graph_init.size());

  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;
  double sigma_dynamics = 1e-5;  // std of dynamics constraints.
  auto opt = gtdynamics::OptimizerSetting(sigma_dynamics);
  auto graph_builder = gtdynamics::DynamicsGraph(opt, gravity);
  vector<gtsam::NonlinearFactorGraph> transition_graphs =
      trajectory.getTransitionGraphs(graph_builder, mu);
  EXPECT_LONGS_EQUAL(repeat * 2 - 1, transition_graphs.size());
  // regression test
  EXPECT_LONGS_EQUAL(205, transition_graphs[0].size());

  // Test multi-phase factor graph.
  auto graph = trajectory.multiPhaseFactorGraph(
      graph_builder, gtdynamics::CollocationScheme::Euler, mu);
  // regression test
  EXPECT_LONGS_EQUAL(4330, graph.size());
  EXPECT_LONGS_EQUAL(4712, graph.keys().size());

  Values init_vals = trajectory.multiPhaseInitialValues(1e-5, 1. / 240);
  EXPECT_LONGS_EQUAL(4712, init_vals.size());

  // Test objectives for contact links.
  const double ground_height = 0.0;
  auto contact_link_objectives = trajectory.contactLinkObjectives(
      noiseModel::Isotropic::Sigma(3, 1e-7), ground_height);
  // regression test
  EXPECT_LONGS_EQUAL(80, contact_link_objectives.size());
  // regression
  auto last_factor = boost::dynamic_pointer_cast<gtdynamics::PointGoalFactor>(
      contact_link_objectives.back());
  EXPECT(gtsam::assert_equal(gtsam::Point3(3.00478, -1.33761, 0),
                             last_factor->goalPoint(), 1e-5));

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
