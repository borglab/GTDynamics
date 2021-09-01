/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSpiderWalking.cpp
 * @brief Test robot trajectory optimization with Phases.
 * @author: Alejandro Escontrela, Stephanie McCormick
 * @author: Disha Das, Tarushree Gandhi
 * @author: Frank Dellaert, Varun Agrawal, Stefanos Charalambous
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Unit;
using std::string;
using std::vector;

using namespace gtdynamics;

// Returns a Trajectory object for a single robot walk cycle.
Trajectory getTrajectory(const Robot &robot, size_t repeat) {
  vector<LinkSharedPtr> odd_links = {robot.link("tarsus_1_L1"),
                                     robot.link("tarsus_3_L3"),
                                     robot.link("tarsus_5_R4"),
                                     robot.link("tarsus_7_R2")};
  vector<LinkSharedPtr> even_links = {robot.link("tarsus_2_L2"),
                                      robot.link("tarsus_4_L4"),
                                      robot.link("tarsus_6_R3"),
                                      robot.link("tarsus_8_R1")};
  auto links = odd_links;
  links.insert(links.end(), even_links.begin(), even_links.end());

  const Point3 contact_in_com(0, 0.19, 0);
  Phase stationary(1, links, contact_in_com), odd(2, odd_links, contact_in_com),
      even(2, even_links, contact_in_com);

  WalkCycle walk_cycle;
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(even);
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(odd);

  return Trajectory(walk_cycle, repeat);
}

TEST(testSpiderWalking, WholeEnchilada) {
  // Load Stephanie's robot robot (alt version, created by Tarushree/Disha).
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider_alt.sdf"), "spider");

  double sigma_dynamics = 1e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-6;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  OptimizerSetting opt(sigma_dynamics);
  DynamicsGraph graph_builder(opt, gravity);

  // Create the trajectory, consisting of 2 walk phases, each consisting of 4
  // phases: [stationary, odd, stationary, even].
  auto trajectory = getTrajectory(robot, 2);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  auto graph =
      trajectory.multiPhaseFactorGraph(robot, graph_builder, collocation, mu);
  EXPECT_LONGS_EQUAL(3557, graph.size());
  EXPECT_LONGS_EQUAL(3847, graph.keys().size());

  // Build the objective factors.
  const Point3 step(0, 0.4, 0);
  NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(robot, Isotropic::Sigma(3, 1e-7), step);
  // per walk cycle: 1*8 + 2*8 + 1*8 + 2*8 = 48
  // 2 repeats, hence:
  EXPECT_LONGS_EQUAL(48 * 2, objectives.size());
  EXPECT_LONGS_EQUAL(48 * 2, objectives.keys().size());

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add base goal objectives to the factor graph.
  auto base_link = robot.link("body");
  for (int k = 0; k <= K; k++) {
    objectives.add(
        LinkObjectives(base_link->id(), k)
            .pose(Pose3(Rot3(), Point3(0, 0.0, 0.5)), Isotropic::Sigma(6, 5e-5))
            .twist(gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5)));
  }

  // Add link and joint boundary conditions to FG.
  trajectory.addBoundaryConditions(&objectives, robot, dynamics_model_6,
                                   dynamics_model_6, objectives_model_6,
                                   objectives_model_1, objectives_model_1);

  // Constrain all Phase keys to have duration of 1 /240.
  const double desired_dt = 1. / 240;
  trajectory.addIntegrationTimeFactors(&objectives, desired_dt, 1e-30);

  // Add min torque objectives.
  trajectory.addMinimumTorqueFactors(&objectives, robot, Unit::Create(1));

  // Add prior on hip joint angles (spider specific)
  auto prior_model = Isotropic::Sigma(1, 1.85e-4);
  for (auto &&joint : robot.joints())
    if (joint->name().find("hip2") == 0)
      for (int k = 0; k <= K; k++)
        objectives.add(JointObjectives(joint->id(), k).angle(2.5, prior_model));

  // Regression test on objective factors
  EXPECT_LONGS_EQUAL(910, objectives.size());
  EXPECT_LONGS_EQUAL(899, objectives.keys().size());

  // Add objective factors to the graph
  graph.add(objectives);
  EXPECT_LONGS_EQUAL(3557 + 910, graph.size());
  EXPECT_LONGS_EQUAL(3847, graph.keys().size());

  // Initialize solution.
  double gaussian_noise = 1e-5;
  Values init_vals =
      trajectory.multiPhaseInitialValues(robot, gaussian_noise, desired_dt);
  EXPECT_LONGS_EQUAL(3847, init_vals.size());

  // Optimize!
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals);
  auto results = optimizer.optimize();

  // TODO(frank): test whether it works
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
