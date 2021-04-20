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

#define GROUND_HEIGHT -1.75  //-1.75

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
  vector<string> odd_links{"tarsus_1", "tarsus_3", "tarsus_5", "tarsus_7"};
  vector<string> even_links{"tarsus_2", "tarsus_4", "tarsus_6", "tarsus_8"};
  auto links = odd_links;
  links.insert(links.end(), even_links.begin(), even_links.end());

  Phase stationary(robot, 1);
  stationary.addContactPoints(links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  Phase odd(robot, 2);
  odd.addContactPoints(odd_links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  Phase even(robot, 2);
  even.addContactPoints(even_links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  WalkCycle walk_cycle;
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(even);
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(odd);

  Trajectory trajectory(walk_cycle, repeat);
  return trajectory;
}

TEST(testSpiderWalking, WholeEnchilada) {
  // Load Stephanie's robot robot (alt version, created by Tarushree/Disha).
  Robot robot =
      CreateRobotFromFile(SDF_PATH + "/test/spider_alt.sdf", "spider");

  double sigma_dynamics = 1e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-6;  // std of additional objectives.
  double sigma_joints = 1.85e-4;   // 1.85e-4

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       dynamics_model_1_2 = Isotropic::Sigma(1, sigma_joints),
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
  auto collocation = DynamicsGraph::CollocationScheme::Euler;
  auto graph = trajectory.multiPhaseFactorGraph(graph_builder, collocation, mu);
  EXPECT_LONGS_EQUAL(3443, graph.size());
  EXPECT_LONGS_EQUAL(3819, graph.keys().size());

  // Build the objective factors.
  NonlinearFactorGraph objective_factors = trajectory.contactLinkObjectives(
      Isotropic::Sigma(3, 1e-7), GROUND_HEIGHT);
  // Regression test on objective factors
  EXPECT_LONGS_EQUAL(104, objective_factors.size());
  EXPECT_LONGS_EQUAL(104, objective_factors.keys().size());

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add base goal objectives to the factor graph.
  auto base_link = robot.link("body");
  for (int k = 0; k <= K; k++) {
    add_link_objective(&objective_factors, Pose3(Rot3(), Point3(0, 0.0, 0.5)),
                       Isotropic::Sigma(6, 5e-5), gtsam::Z_6x1,
                       Isotropic::Sigma(6, 5e-5), base_link->id(), k);
  }

  // Add link and joint boundary conditions to FG.
  trajectory.addBoundaryConditions(&objective_factors, robot, dynamics_model_6,
                                   dynamics_model_6, objectives_model_6,
                                   objectives_model_1, objectives_model_1);

  // Add prior on hip joint angles
  for (auto &&joint : robot.joints()) {
    if (joint->name().find("hip2") == 0) {
      const int id = joint->id();
      // Add priors to joint angles
      for (int k = 0; k <= K; k++) {
        add_joint_objective(&objective_factors, 2.5, dynamics_model_1_2, id, k);
      }
    }
  }

  // Constrain all Phase keys to have duration of 1 /240.
  const double desired_dt = 1. / 240;
  trajectory.addIntegrationTimeFactors(&objective_factors, desired_dt);

  // Add min torque objectives.
  trajectory.addMinimumTorqueFactors(&objective_factors, robot,
                                     Unit::Create(1));

  // Regression test on objective factors
  EXPECT_LONGS_EQUAL(918, objective_factors.size());
  EXPECT_LONGS_EQUAL(907, objective_factors.keys().size());

  // Add objective factors to the graph
  graph.add(objective_factors);
  EXPECT_LONGS_EQUAL(3443 + 918, graph.size());
  EXPECT_LONGS_EQUAL(3819, graph.keys().size());

  // Initialize solution.
  double gaussian_noise = 1e-5;
  Values init_vals =
      trajectory.multiPhaseInitialValues(gaussian_noise, desired_dt);

  // Check all the keys
  auto graph_keys = graph.keys();
  for (auto &&key : init_vals.keys()) {
    if (!graph_keys.exists(key)) {
      std::cout << _GTDKeyFormatter(key) << std::endl;
    }
  }

  EXPECT_LONGS_EQUAL(3819, init_vals.size());  // says it's 3847

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e0);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // TODO(frank): test whether it works
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
