/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSpiderWalking.cpp
 * @brief Test spider trajectory optimization with Phases.
 * @author: Alejandro Escontrela, Stephanie McCormick
 * @author: Disha Das, Tarushree Gandhi
 * @author: Frank Dellaert, Varun Agrawal, Stefanos Charalambous
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/WalkCycle.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#define GROUND_HEIGHT -1.75  //-1.75

using gtdynamics::ContactPoint;
using gtdynamics::ContactPoints;
using gtdynamics::Phase;
using gtdynamics::Robot;
using gtdynamics::ZeroValues;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;
using gtsam::noiseModel::Isotropic;
using std::string;
using std::vector;

using namespace gtsam;
using namespace gtdynamics;

// Returns a Trajectory object for a single spider walk cycle.
gtdynamics::Trajectory getTrajectory(const Robot &spider, size_t repeat) {
  vector<string> odd_links{"tarsus_1", "tarsus_3", "tarsus_5", "tarsus_7"};
  vector<string> even_links{"tarsus_2", "tarsus_4", "tarsus_6", "tarsus_8"};
  auto links = odd_links;
  links.insert(links.end(), even_links.begin(), even_links.end());

  gtdynamics::Phase stationary(spider, 4);
  stationary.addContactPoints(links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  gtdynamics::Phase odd(spider, 2);
  odd.addContactPoints(odd_links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  gtdynamics::Phase even(spider, 2);
  even.addContactPoints(even_links, Point3(0, 0.19, 0), GROUND_HEIGHT);

  gtdynamics::WalkCycle walk_cycle;
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(even);
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(odd);

  gtdynamics::Trajectory trajectory(walk_cycle, repeat);
  return trajectory;
}

TEST(testSpiderWalking, WholeEnchilada) {
  // Load Stephanie's spider robot (alt version, created by Tarushree/Disha).
  Robot spider = gtdynamics::CreateRobotFromFile(
      SDF_PATH + "/test/spider_alt.sdf", "spider");

  double sigma_dynamics = 1e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-6;  // std of additional objectives.
  double sigma_joints = 1.85e-4;   // 1.85e-4

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_3 = Isotropic::Sigma(3, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       dynamics_model_1_2 = Isotropic::Sigma(1, sigma_joints),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_3 = Isotropic::Sigma(3, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  gtdynamics::OptimizerSetting opt(sigma_dynamics);
  gtdynamics::DynamicsGraph graph_builder(opt, gravity);

  // Create the trajectory, consisting of 2 walk phases, each consisting of 4
  // phases: [stationary, odd, stationary, even].
  auto trajectory = getTrajectory(spider, 2);

  // Create multi-phase trajectory factor graph
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;
  auto graph = trajectory.multiPhaseFactorGraph(graph_builder, collocation, mu);
  LONGS_EQUAL(6551, graph.size());
  LONGS_EQUAL(7311, graph.keys().size());

  // Build the objective factors.
  NonlinearFactorGraph objective_factors = trajectory.contactLinkObjectives(
      Isotropic::Sigma(3, 1e-7), GROUND_HEIGHT);
  // Regression test on objective factors
  LONGS_EQUAL(200, objective_factors.size());
  LONGS_EQUAL(200, objective_factors.keys().size());

  // TODO(frank): the fact that I'm doing this is a red flag
  using gtdynamics::PhaseKey;
  using gtdynamics::internal::JointAccelKey;
  using gtdynamics::internal::JointAngleKey;
  using gtdynamics::internal::JointVelKey;
  using gtdynamics::internal::PoseKey;
  using gtdynamics::internal::TorqueKey;
  using gtdynamics::internal::TwistAccelKey;
  using gtdynamics::internal::TwistKey;

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add base goal objectives to the factor graph.
  auto base_link = spider.link("body");
  for (int k = 0; k <= K; k++) {
    gtdynamics::add_link_objective(
        &objective_factors, Pose3(Rot3(), Point3(0, 0.0, 0.5)),
        Isotropic::Sigma(6, 5e-5), gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5),
        base_link->id(), k);
  }

  // Add link boundary conditions to FG.
  for (auto &&link : spider.links()) {
    // Initial link pose, twists.
    gtdynamics::add_link_objective(&objective_factors, link->wTcom(),
                                   dynamics_model_6, Vector6::Zero(),
                                   dynamics_model_6, link->id(), 0);

    // Final link twists, accelerations.
    gtdynamics::add_twist_objective(&graph, gtsam::Z_6x1, objectives_model_6,
                                    gtsam::Z_6x1, objectives_model_6,
                                    link->id(), K);
  }

  // Add joint boundary conditions to FG.
  for (auto &&joint : spider.joints()) {
    // Add priors to joint angles
    for (int t = 0; t <= K; t++) {
      if (joint->name().find("hip2") == 0)
        objective_factors.add(PriorFactor<double>(JointAngleKey(joint->id(), t),
                                                  2.5, dynamics_model_1_2));
    }
    objective_factors.add(PriorFactor<double>(JointVelKey(joint->id(), 0), 0.0,
                                              dynamics_model_1));
    objective_factors.add(PriorFactor<double>(JointVelKey(joint->id(), K), 0.0,
                                              objectives_model_1));
    objective_factors.add(PriorFactor<double>(JointAccelKey(joint->id(), K),
                                              0.0, objectives_model_1));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 /240.
  double desired_dt = 1. / 240;
  for (int phase = 0; phase < trajectory.numPhases(); phase++)
    objective_factors.add(PriorFactor<double>(
        PhaseKey(phase), desired_dt, noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= K; t++) {
    for (auto &&joint : spider.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->id(), t), noiseModel::Gaussian::Covariance(I_1x1)));
  }

  // Regression test on objective factors
  LONGS_EQUAL(200 + 1286, objective_factors.size());
  LONGS_EQUAL(1475, objective_factors.keys().size());

  // Add objective factors to the graph
  graph.add(objective_factors);
  LONGS_EQUAL(6551 + 200 + 1286, graph.size());
  LONGS_EQUAL(7311, graph.keys().size());

  // TODO: Pass Trajectory here
  // Initialize solution.
  // phase transition initial values.
  double gaussian_noise = 1e-5;
  vector<Values> transition_graph_init =
      trajectory.getInitTransitionValues(gaussian_noise);
  auto robots = trajectory.phaseRobotModels();
  auto phase_durations = trajectory.phaseDurations();
  auto phase_cps = trajectory.phaseContactPoints();
  Values init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
      robots, phase_durations, transition_graph_init, desired_dt,
      gaussian_noise, phase_cps);
  LONGS_EQUAL(7311, init_vals.size());  // says it's 7435

  // Optimize!
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e0);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // TODO(frank): test whether it works
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
