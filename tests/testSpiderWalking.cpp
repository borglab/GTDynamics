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
gtdynamics::Trajectory getTrajectory(const Robot &spider, size_t repeat = 2) {
  vector<string> odd_links{"tarsus_1", "tarsus_3", "tarsus_5", "tarsus_7"};
  vector<string> even_links{"tarsus_2", "tarsus_4", "tarsus_6", "tarsus_8"};
  auto links = odd_links;
  links.insert(links.end(), even_links.begin(), even_links.end());

  gtdynamics::Phase stationary(spider, 4);
  stationary.addContactPoints(links, gtsam::Point3(0, 0.19, 0), GROUND_HEIGHT);

  gtdynamics::Phase odd(spider, 2);
  odd.addContactPoints(odd_links, gtsam::Point3(0, 0.19, 0), GROUND_HEIGHT);

  gtdynamics::Phase even(spider, 2);
  even.addContactPoints(even_links, gtsam::Point3(0, 0.19, 0), GROUND_HEIGHT);

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

  vector<string> links = {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4",
                          "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"};
  auto spider_trajectory = getTrajectory(spider);

  // Define noise to be added to initial values, desired timestep duration,
  // vector of link name strings, robot model for each phase, and

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

  // Create multi-phase trajectory factor graph
  auto graph =
      spider_trajectory.multiPhaseFactorGraph(graph_builder, collocation, mu);

  // Build the objective factors.
  gtsam::NonlinearFactorGraph objective_factors;
  auto base_link = spider.link("body");

  std::map<string, gtdynamics::LinkSharedPtr> link_map;
  for (auto &&link : links)
    link_map.insert(std::make_pair(link, spider.link(link)));

  // Previous contact point goal.
  std::map<string, Point3> prev_cp = spider_trajectory.initContactPointGoal();

  // Distance to move contact point per time step during swing.
  auto contact_offset = Point3(0, 0.02, 0);

  // Add contact point objectives to factor graph.
  for (int p = 0; p < spider_trajectory.numPhases(); p++) {
    // if(p <2) contact_offset /=2 ;
    // Phase start and end timesteps.
    int t_p_i = spider_trajectory.getStartTimeStep(p);
    int t_p_f = spider_trajectory.getEndTimeStep(p);

    // Obtain the contact links and swing links for this phase.
    vector<string> phase_contact_links =
        spider_trajectory.getPhaseContactLinks(p);
    vector<string> phase_swing_links = spider_trajectory.getPhaseSwingLinks(p);

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

      for (auto &&pcl : phase_contact_links)
        objective_factors.add(spider_trajectory.pointGoalFactor(
            pcl, t, Isotropic::Sigma(3, 1e-7),  // 1e-7
            Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.05)));

      double h =
          GROUND_HEIGHT + std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto &&psl : phase_swing_links)
        objective_factors.add(spider_trajectory.pointGoalFactor(
            psl, t, Isotropic::Sigma(3, 1e-7),
            Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));

      // Update the goal point for the swing links.
      for (auto &&psl : phase_swing_links)
        prev_cp[psl] = prev_cp[psl] + contact_offset;
    }
  }

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
  int t_f = spider_trajectory.getEndTimeStep(spider_trajectory.numPhases() - 1);

  // Add base goal objectives to the factor graph.
  for (int t = 0; t <= t_f; t++) {
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(base_link->id(), t),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.0, 0.5)),  // 0.5
        Isotropic::Sigma(6, 5e-5)));  // 6.2e-5 //5e-5
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(base_link->id(), t), Vector6::Zero(),
        Isotropic::Sigma(6, 5e-5)));
  }

  // Add link boundary conditions to FG.
  for (auto &&link : spider.links()) {
    // Initial link pose, twists.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(link->id(), 0), link->wTcom(), dynamics_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->id(), 0), Vector6::Zero(), dynamics_model_6));

    // Final link twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->id(), t_f), Vector6::Zero(), objectives_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistAccelKey(link->id(), t_f), Vector6::Zero(), objectives_model_6));
  }

  // Add joint boundary conditions to FG.
  for (auto &&joint : spider.joints()) {
    // Add priors to joint angles
    for (int t = 0; t <= t_f; t++) {
      if (joint->name().find("hip2") == 0)
        objective_factors.add(gtsam::PriorFactor<double>(
            JointAngleKey(joint->id(), t), 2.5, dynamics_model_1_2));
    }
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->id(), 0), 0.0, dynamics_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->id(), t_f), 0.0, objectives_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAccelKey(joint->id(), t_f), 0.0, objectives_model_1));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 /240.
  double desired_dt = 1. / 240;
  for (int phase = 0; phase < spider_trajectory.numPhases(); phase++)
    objective_factors.add(gtsam::PriorFactor<double>(
        PhaseKey(phase), desired_dt,
        gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto &&joint : spider.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->id(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);
  GTD_PRINT(graph);
  LONGS_EQUAL(8037, graph.size());
  LONGS_EQUAL(7311, graph.keys().size());

  // TODO: Pass Trajectory here
  // Initialize solution.
  // phase transition initial values.
  double gaussian_noise = 1e-5;
  vector<Values> transition_graph_init =
      spider_trajectory.getInitTransitionValues(gaussian_noise);
  auto robots = spider_trajectory.phaseRobotModels();
  auto phase_durations = spider_trajectory.phaseDurations();
  auto phase_cps = spider_trajectory.phaseContactPoints();
  gtsam::Values init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
      robots, phase_durations, transition_graph_init, desired_dt,
      gaussian_noise, phase_cps);
  GTD_PRINT(init_vals);
  LONGS_EQUAL(7311, init_vals.size());

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
