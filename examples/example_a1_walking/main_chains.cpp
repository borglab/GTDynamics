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
#include <gtdynamics/dynamics/LeanDynamicsGraph.h>
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

  /*vector<LinkSharedPtr> all_but_rl = {robot.link("FL_lower"),robot.link("FR_lower"),robot.link("RR_lower")};
  vector<LinkSharedPtr> all_but_rr = {robot.link("FL_lower"),robot.link("FR_lower"),robot.link("RL_lower")};
  vector<LinkSharedPtr> all_but_fl = {robot.link("RL_lower"),robot.link("FR_lower"),robot.link("RR_lower")};
  vector<LinkSharedPtr> all_but_fr = {robot.link("FL_lower"),robot.link("RL_lower"),robot.link("RR_lower")};
  vector<LinkSharedPtr> all_feet = {robot.link("FL_lower"),robot.link("FR_lower"),robot.link("RR_lower"), robot.link("RL_lower")};*/

  // Create three different FootContactConstraintSpecs, one for all the feet on the
  // ground, one with RL and FR, one with RR and FL
  const Point3 contact_in_com(0, 0, -0.07);
  auto stationary = boost::make_shared<FootContactConstraintSpec>(all_feet, contact_in_com);
  auto RLFR = boost::make_shared<FootContactConstraintSpec>(rlfr, contact_in_com);
  auto RRFL = boost::make_shared<FootContactConstraintSpec>(rrfl, contact_in_com);

  /*const Point3 contact_in_com(0, 0, -0.07);
  auto stationary = boost::make_shared<FootContactConstraintSpec>(all_feet, contact_in_com);
  auto rl = boost::make_shared<FootContactConstraintSpec>(all_but_rl, contact_in_com);
  auto rr = boost::make_shared<FootContactConstraintSpec>(all_but_rr, contact_in_com);
  auto fl = boost::make_shared<FootContactConstraintSpec>(all_but_fl, contact_in_com);
  auto fr = boost::make_shared<FootContactConstraintSpec>(all_but_fr, contact_in_com);*/

  //FootContactVector states = {noRL, noRR, noFR, noFL};
  FootContactVector states = {stationary, RRFL, stationary, RLFR};
  std::vector<size_t> phase_lengths = {5, 30, 5, 30};


  /*FootContactVector states = {stationary, rl, stationary, rr, stationary, rl, stationary, rr};
  std::vector<size_t> phase_lengths = {15,5,15,5,15,5,15,5};*/

  WalkCycle walk_cycle(states, phase_lengths);

  return Trajectory(walk_cycle, repeat);
}

std::vector<std::vector<JointSharedPtr>> getChainJoints(const Robot& robot) {

  std::vector<JointSharedPtr> FR,FL,RR,RL;

  for (auto&& joint : robot.joints()) {
    if (joint->name().find("FR") < 100) {
      FR.emplace_back(joint);
    }
    if (joint->name().find("FL") < 100) {
      FL.emplace_back(joint);
    }
    if (joint->name().find("RR") < 100) {
      RR.emplace_back(joint);
    }
    if (joint->name().find("RL") < 100) {
      RL.emplace_back(joint);
    }
  }

  std::swap(FR[1],FR[2]);
  std::swap(FL[1],FL[2]);
  std::swap(RR[1],RR[2]);
  std::swap(RL[1],RL[2]);
  std::vector<std::vector<JointSharedPtr>> chain_joints{FR,FL,RR,RL};
  
  return chain_joints;
}

std::vector<Chain> getComposedChains(const std::vector<std::vector<JointSharedPtr>> &chain_joints) {

  std::vector<Chain> fr_chain, fl_chain, rr_chain, rl_chain;

  for (auto&& joint : chain_joints[0]) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    fr_chain.emplace_back(single_link_chain);
  }
  for (auto&& joint : chain_joints[1]) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    fl_chain.emplace_back(single_link_chain);
  }
  for (auto&& joint : chain_joints[2]) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    rr_chain.emplace_back(single_link_chain);
  }
  for (auto&& joint : chain_joints[3]) {
    Chain single_link_chain(joint->pMc(), joint->cScrewAxis());
    rl_chain.emplace_back(single_link_chain);
  }

  // Compose chains
  Chain composed_fr = Chain::compose(fr_chain);
  Chain composed_fl = Chain::compose(fl_chain);
  Chain composed_rr = Chain::compose(rr_chain);
  Chain composed_rl = Chain::compose(rl_chain);

  std::vector<Chain> composed_chains{composed_fr, composed_fl, composed_rr, composed_rl};

  return composed_chains;
}

int main(int argc, char** argv) {

  gttic_(start); //need to build gtsam with timing for this

  // Load Unitree A1 robot urdf.
  auto robot =
      CreateRobotFromFile(std::string("/home/dan/Desktop/Projects/GTDynamics/models/urdfs/a1/a1.urdf"), "a1");

  for (auto&& link : robot.links()) {
    if (link->name().find("trunk") > 100) {
        link->setMassValue(0);
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

  // Create the trajectory, 1 walk cycle.
  auto trajectory = getTrajectory(robot, 1);

  // Prepare inputs for LeanDynamicsGraph
  auto chain_joints = getChainJoints(robot);
  auto composed_chains = getComposedChains(chain_joints);

  OptimizerSetting opt(sigma_dynamics);
  gtsam::Vector3 tolerance(1e-8, 1e-8, 1e-8);
  LeanDynamicsGraph graph_builder(opt, composed_chains, chain_joints, tolerance, gravity);

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
  /*prior_model = Isotropic::Sigma(1, 1e-3);
  for (auto&& joint : robot.joints())
    if (joint->name().find("upper") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("hip") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(1.0, prior_model));
      }
  // Add objectives to factor graph.
   graph.add(objectives);*/


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

  /*for ( auto&& key: init_vals.keys()) {
    auto a = graph.keys().exists(key);
    if (a==0) { 
      std::cout << key << std::endl;
    }
  }*/

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  //params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e10);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  graph_builder.saveGraphMultiSteps("/home/dan/Desktop/Projects/GTDynamics/build/factor_graph_a1.json", graph,
                                    results, robot, 4, false);

  gttoc_(optimization);
  gttoc_(start);

  gtsam::tictoc_print_(); //need to build gtsam with timing for this
  // Write results to traj file
  trajectory.writeToFile(robot, "a1_traj.csv", results);

  return 0;
}
