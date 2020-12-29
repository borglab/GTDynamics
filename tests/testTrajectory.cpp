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
#include "gtdynamics/utils/Trajectory.h"
#include "gtdynamics/utils/WalkCycle.h"
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

using namespace gtdynamics;
using namespace gtsam;

// Class to test protected method
class TrajectoryTest : public Trajectory {
 public:
  TrajectoryTest() : Trajectory(){};

  const ContactPoints getIntersection(ContactPoints CPs_1,
                                      ContactPoints CPs_2) const {
    return Trajectory::getIntersection(CPs_1, CPs_2);
  }
};

TEST(Trajectory, Intersection) {
  Robot robot_configuration =
      CreateRobotFromFile(std::string(SDF_PATH) + "/test/spider.sdf", "spider");

  double contact_height = 5;
  size_t num_time_steps = 1;

  // Initialize first phase
  auto phase_1 = gtdynamics::Phase(robot_configuration, num_time_steps);
  phase_1.addContactPoint("tarsus_2", gtsam::Point3(3, 3, 3), contact_height);
  phase_1.addContactPoint("tarsus_1", gtsam::Point3(3, 3, 3), contact_height);
  phase_1.addContactPoint("tarsus_3", gtsam::Point3(3, 3, 3), contact_height);

  // Initialize second phase
  auto phase_2 = gtdynamics::Phase(robot_configuration, num_time_steps);
  phase_2.addContactPoint("tarsus_3", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_4", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_5", gtsam::Point3(3, 3, 3), contact_height);
  phase_2.addContactPoint("tarsus_2", gtsam::Point3(3, 3, 3), contact_height);

  TrajectoryTest traj;
  ContactPoints intersection = traj.getIntersection(
      phase_1.getAllContactPoints(), phase_2.getAllContactPoints());

  ContactPoints expected = {
      {"tarsus_2", ContactPoint{Point3(3, 3, 3), 0, contact_height}},
      {"tarsus_3", ContactPoint{Point3(3, 3, 3), 0, contact_height}}};

  for (auto const& contact_point : intersection) {
    EXPECT(expected[contact_point.first] == contact_point.second);
  }
}

TEST(Trajectory, error) {
  Robot robot_configuration =
      CreateRobotFromFile(std::string(SDF_PATH) + "/test/spider.sdf", "spider");

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

  // Initialize Trajectory
  size_t repeat = 5;
  auto trajectory = gtdynamics::Trajectory(walk_cycle, repeat);

  auto phase_cps = trajectory.phaseContactPoints();
  EXPECT(phase_cps.size() == 10);
  EXPECT(phase_cps[2].size() == 3);

  auto trans_cps = trajectory.transitionContactPoints();
  EXPECT(trans_cps.size() == 9);
  EXPECT(trans_cps[1].size() == 2);

  auto phase_durations = trajectory.phaseDurations();
  EXPECT(phase_durations[2] == 20);

  auto robot_models = trajectory.phaseRobotModels();
  EXPECT(robot_models.size() == 10);

  auto final_timesteps = trajectory.finalTimeSteps();
  EXPECT(final_timesteps[2] == 65);
  EXPECT(trajectory.getStartTimeStep(2) == 46);
  EXPECT(trajectory.getEndTimeStep(2) == 65);
  EXPECT(trajectory.getLinks().size() == 5);
  EXPECT(trajectory.getPhaseContactLinks(3).size() == 4);
  EXPECT(trajectory.getPhaseSwingLinks(3).size() == 1);

  auto prev_cp = trajectory.initContactPointGoal();
  EXPECT(prev_cp.size() == 5);

  double gaussian_noise = 1e-5;
  vector<Values> transition_graph_init =
      trajectory.getInitTransitionValues(gaussian_noise);
  EXPECT(transition_graph_init.size() == 9);

  gtsam::Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;
  double sigma_dynamics = 1e-5;  // std of dynamics constraints.
  auto opt = gtdynamics::OptimizerSetting(sigma_dynamics);
  auto graph_builder = gtdynamics::DynamicsGraph(opt);
  vector<gtsam::NonlinearFactorGraph> transition_graphs =
      trajectory.getTransitionGraphs(graph_builder, gravity, mu);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
