/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Unitree A1 trajectory optimization with pre-specified footholds.
 * @author: Dan Barladeanu
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/timing.h>

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

// Returns a Trajectory object for a single a1 walk cycle.
Trajectory getTrajectory(const Robot& robot, size_t repeat) {
  vector<LinkSharedPtr> rlfr = {robot.link("RL_lower"),robot.link("FR_lower")};
  vector<LinkSharedPtr> rrfl = {robot.link("RR_lower"),robot.link("FL_lower")};
  auto all_feet = rlfr;
  all_feet.insert(all_feet.end(), rrfl.begin(), rrfl.end());

  // Create three different FootContactConstraintSpecs, one for all the feet on the
  // ground, one with RL and FR, one with RR and FL
  const Point3 contact_in_com(0, 0, -0.07);
  auto stationary = boost::make_shared<FootContactConstraintSpec>(all_feet, contact_in_com);
  auto RLFR = boost::make_shared<FootContactConstraintSpec>(rlfr, contact_in_com);
  auto RRFL = boost::make_shared<FootContactConstraintSpec>(rrfl, contact_in_com);

  //FootContactVector states = {noRL, noRR, noFR, noFL};
  FootContactVector states = {stationary, RRFL, stationary, RLFR};
  std::vector<size_t> phase_lengths = {5, 30, 5, 30};

  WalkCycle walk_cycle(states, phase_lengths);

  return Trajectory(walk_cycle, repeat);
}

int main(int argc, char** argv) {

  gttic_(start); //need to build gtsam with timing for this

  // Load Unitree A1 robot urdf.
  auto robot =
      CreateRobotFromFile(std::string("/home/dan/Desktop/Projects/GTDynamics/models/urdfs/a1/a1.urdf"), "a1");


  for (auto&& link : robot.links()) {
    if (link->name().find("trunk") > 100) {
        //link->setMassValue(0);
        //link->setInertiaZero();
        std::cout << link->name() << std::endl;
    }
  }

  double sigma_dynamics = 1e-6;    // std of dynamics constraints.
  double sigma_objectives = 1e-7;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  OptimizerSetting opt(sigma_dynamics);
  DynamicsGraph graph_builder(opt, gravity);

  // Create the trajectory, 1 walk cycle.
  auto trajectory = getTrajectory(robot, 1);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  auto graph =
      trajectory.multiPhaseFactorGraph(robot, graph_builder, collocation, mu);

  // Build the objective factors.
  double ground_height = 1.0;
  const Point3 step(0.1, 0, 0);
  gtsam::NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(robot, Isotropic::Sigma(3, 1e-7), step, ground_height);

  // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add body goal objectives to the factor graph.
  auto base_link = robot.link("trunk");
  for (int k = 0; k <= K; k++) {
    objectives.add(
        LinkObjectives(base_link->id(), k)
            .pose(Pose3(Rot3(), Point3(0, 0, 0.4)), Isotropic::Sigma(6, 1e-5))
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

  // Add prior on A1 lower joint angles
  auto prior_model = Isotropic::Sigma(1, 1e-6);
  for (auto&& joint : robot.joints())
    if (joint->name().find("lower") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("lower") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(-1.4, prior_model));
      }
  // Add objectives to factor graph.
  graph.add(objectives);

  // Add prior on A1 hip joint angles
  prior_model = Isotropic::Sigma(1, 1e-7);
  for (auto&& joint : robot.joints())
    if (joint->name().find("hip") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("hip") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(0.0, prior_model));
      }
  // Add objectives to factor graph.
   graph.add(objectives);
 
  // Initialize solution.
  double gaussian_noise = 1e-3;
 Initializer initializer;
  gtsam::Values init_vals =
      trajectory.multiPhaseInitialValues(robot, initializer, gaussian_noise, desired_dt);

  gttic_(optimization);

  std::cout << "graph size = " << graph.size() << std::endl;
  std::cout << "graph keys size = " << graph.keys().size() << std::endl;
  std::cout << "init vals size = " << init_vals.size() << std::endl;
  
  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  //params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e10);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  gttoc_(optimization);
  gttoc_(start);

  gtsam::tictoc_print_(); //need to build gtsam with timing for this
  // Write results to traj file
  trajectory.writeToFile(robot, "a1_traj.csv", results);

  return 0;
}
