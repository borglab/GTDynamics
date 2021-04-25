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
Trajectory getTrajectory(vector<string> links, Robot robot, size_t repeat) {
  const Point3 contact_in_com(0, 0.19, 0);
  Phase stationary(40);
  stationary.addContactPoints(links, contact_in_com);

  Phase odd(20);
  odd.addContactPoints(
      {{"tarsus_1_L1", "tarsus_3_L3", "tarsus_5_R4", "tarsus_7_R2"}},
      contact_in_com);

  Phase even(20);
  even.addContactPoints(
      {{"tarsus_2_L2", "tarsus_4_L4", "tarsus_6_R3", "tarsus_8_R1"}},
      contact_in_com);

  WalkCycle walk_cycle;
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(even);
  walk_cycle.addPhase(stationary);
  walk_cycle.addPhase(odd);

  Trajectory trajectory(robot, walk_cycle, repeat);
  return trajectory;
}

int main(int argc, char **argv) {
  // Load Stephanie's spider robot.
  auto robot =
      CreateRobotFromFile(kSdfPath + string("/spider_alt.sdf"), "spider");

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

  vector<string> links = {"tarsus_1_L1", "tarsus_2_L2", "tarsus_3_L3",
                          "tarsus_4_L4", "tarsus_5_R4", "tarsus_6_R3",
                          "tarsus_7_R2", "tarsus_8_R1"};

  // Create the trajectory, consisting of 3 walk cycles, each consisting of 4
  // phases: [stationary, odd, stationary, even].
  auto trajectory = getTrajectory(links, robot, 3);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  auto graph = trajectory.multiPhaseFactorGraph(graph_builder, collocation, mu);

  // Build the objective factors.
  const Point3 step(0, 0.4, 0);
  gtsam::NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(Isotropic::Sigma(3, 1e-7), step);

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add base goal objectives to the factor graph.
  auto base_link = robot.link("body");
  for (int k = 0; k <= K; k++) {
    add_link_objectives(&objectives, base_link->id(), k)
        .pose(Pose3(Rot3(), Point3(0, 0.0, 0.5)), Isotropic::Sigma(6, 5e-5))
        .twist(gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5));
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
        add_joint_objectives(&objectives, joint->id(), k)
            .angle(2.5, prior_model);

  // Initialize solution.
  double gaussian_noise = 1e-5;
  gtsam::Values init_vals =
      trajectory.multiPhaseInitialValues(gaussian_noise, desired_dt);

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
  vector<string> jnames;
  for (auto &&joint : robot.joints()) jnames.push_back(joint->name());
  std::cout << jnames.size() << std::endl;
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;

  traj_file.open("forward_traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  for (int phase = 0; phase < trajectory.numPhases(); phase++)
    trajectory.writePhaseToFile(robot, traj_file, results, phase);

  // Write the last 4 phases to disk n times
  for (int i = 0; i < 10; i++) {
    for (int phase = 4; phase < trajectory.numPhases(); phase++)
      trajectory.writePhaseToFile(robot, traj_file, results, phase);
  }
  traj_file.close();
  return 0;
}
