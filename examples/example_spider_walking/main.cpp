/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Spider trajectory optimization with pre-specified footholds.
 * @author: Alejandro Escontrela, Stephanie McCormick, Disha Das, Tarushree
 * Gandhi, Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <utility>

using std::string;
using std::vector;

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector6;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Unit;

using namespace gtdynamics;

// Returns a Trajectory object for a single spider walk cycle.
Trajectory getTrajectory(const Robot& robot, size_t repeat) {
  const vector<LinkSharedPtr> odd_links = {
      robot.link("tarsus_1_L1"), robot.link("tarsus_3_L3"),
      robot.link("tarsus_5_R4"), robot.link("tarsus_7_R2")};
  const vector<LinkSharedPtr> even_links = {
      robot.link("tarsus_2_L2"), robot.link("tarsus_4_L4"),
      robot.link("tarsus_6_R3"), robot.link("tarsus_8_R1")};
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

int main(int argc, char** argv) {
  // Load Stephanie's spider robot.
  auto robot =
      CreateRobotFromFile(kSdfPath + string("spider_alt.sdf"), "spider");

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
  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  OptimizerSetting opt(sigma_dynamics);
  DynamicsGraph graph_builder(opt, gravity);

  // Create the trajectory, consisting of 3 walk cycles, each consisting of 4
  // phases: [stationary, odd, stationary, even].
  auto trajectory = getTrajectory(robot, 3);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  auto graph =
      trajectory.multiPhaseFactorGraph(robot, graph_builder, collocation, mu);

  // Build the objective factors.
  double ground_height = 1.0;
  const Point3 step(0, 0.4, 0);
  gtsam::NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(robot, Isotropic::Sigma(3, 1e-7), step);

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
  for (auto&& joint : robot.joints())
    if (joint->name().find("hip2") == 0)
      for (int k = 0; k <= K; k++)
        objectives.add(JointObjectives(joint->id(), k).angle(2.5, prior_model));

  // Add objectives to factor graph.
  graph.add(objectives);

  // Initialize solution.
  double gaussian_noise = 1e-5;
  gtsam::Values init_vals =
      trajectory.multiPhaseInitialValues(robot, gaussian_noise, desired_dt);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e10);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // Write results to traj file
  trajectory.writeToFile(robot, "forward_traj.csv", results);

  return 0;
}
