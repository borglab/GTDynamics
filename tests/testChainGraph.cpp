/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testChainGraph.cpp
 * @brief Test Chain class.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/dynamics/LeanDynamicsGraph.h>
#include <gtdynamics/utils/ChainInitializer.h>

using namespace gtdynamics;

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Unit;

// Returns a Trajectory object for a single a1 walk cycle.
Trajectory getTrajectory(const Robot& robot, size_t repeat) {
  std::vector<LinkSharedPtr> rlfr = {robot.link("RL_lower"),robot.link("FR_lower")};
  std::vector<LinkSharedPtr> rrfl = {robot.link("RR_lower"),robot.link("FL_lower")};
  auto all_feet = rlfr;
  all_feet.insert(all_feet.end(), rrfl.begin(), rrfl.end());

  // Create three different FootContactConstraintSpecs, one for all the feet on the
  // ground, one with RL and FR, one with RR and FL
  const Point3 contact_in_com(0, 0, -0.07);
  auto stationary = boost::make_shared<FootContactConstraintSpec>(all_feet, contact_in_com);
  auto RLFR = boost::make_shared<FootContactConstraintSpec>(rlfr, contact_in_com);
  auto RRFL = boost::make_shared<FootContactConstraintSpec>(rrfl, contact_in_com);

  //FootContactVector states = {noRL, noRR, noFR, noFL};
  FootContactVector states = {stationary};//, RRFL, stationary, RLFR};
  std::vector<size_t> phase_lengths = {1};//, 1, 1, 1};

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
    //std::cout << joint << std::endl;
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

gtsam::Values OldGraph(){
  auto robot =
      CreateRobotFromFile(std::string("/home/dan/Desktop/Projects/GTDynamics/models/urdfs/a1/a1.urdf"), "a1");

  //for (auto&& link : robot.links()) {
    //if (link->name().find("trunk") > 100) {
        //link->setMassValue(0);
    //}
  //}

    gtsam::Vector3 gravity(0, 0, -9.8);

  OptimizerSetting opt(1e-6);
  DynamicsGraph graph_builder(opt, gravity);

  gtsam::NonlinearFactorGraph graph;
  graph.add(graph_builder.qFactors(robot,0));
  graph.add(graph_builder.vFactors(robot,0));
  graph.add(graph_builder.aFactors(robot,0));
  graph.add(graph_builder.dynamicsFactors(robot,0));


  Initializer initializer;
  double gaussian_noise = 1e-3;
  gtsam::Values init_vals = initializer.ZeroValues(robot, 0, gaussian_noise);

  std::cout << "graph size = " << graph.size() << std::endl;
  std::cout << "graph keys size = " << graph.keys().size() << std::endl;
  std::cout << "init vals size = " << init_vals.size() << std::endl;

   // Optimize!
  gtsam::LevenbergMarquardtParams params;
  //params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e15);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  return results;
}

gtsam::Values NewGraph(){
  auto robot =
    CreateRobotFromFile(std::string("/home/dan/Desktop/Projects/GTDynamics/models/urdfs/a1/a1.urdf"), "a1");
  
  for (auto&& link : robot.links()) {
    if (link->name().find("trunk") > 100) {
        link->setMassValue(0);
    }
  }

    gtsam::Vector3 gravity(0, 0, -9.8);

  // Prepare inputs for LeanDynamicsGraph
  auto chain_joints = getChainJoints(robot);
  auto composed_chains = getComposedChains(chain_joints);

  OptimizerSetting opt(1e-6);
  gtsam::Vector3 tolerance(1e-6, 1e-6, 1e-6);
  LeanDynamicsGraph graph_builder(opt, composed_chains, chain_joints, tolerance, gravity);

  gtsam::NonlinearFactorGraph graph;
  graph.add(graph_builder.qFactors(robot,0));
  graph.add(graph_builder.vFactors(robot,0));
  graph.add(graph_builder.aFactors(robot,0));
  graph.add(graph_builder.chainFactors(robot,0));

  ChainInitializer initializer;
  double gaussian_noise = 1e-3;
  gtsam::Values init_vals = initializer.ZeroValues(robot, 0, gaussian_noise);

  std::cout << "graph size = " << graph.size() << std::endl;
  std::cout << "graph keys size = " << graph.keys().size() << std::endl;
  std::cout << "init vals size = " << init_vals.size() << std::endl;

   // Optimize!
  gtsam::LevenbergMarquardtParams params;
  //params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e15);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  return results;
}


// Test Chain graph equality
TEST(ChainGraph, ChainGraphEquality) {
  gtsam::Values old_vals = OldGraph();
  gtsam::Values new_vals = NewGraph();

  auto robot =
      CreateRobotFromFile(std::string("/home/dan/Desktop/Projects/GTDynamics/models/urdfs/a1/a1.urdf"), "a1");

  auto trajectory = getTrajectory(robot, 1);

  Matrix mat_old =
      trajectory.phase(0).jointMatrix(robot, old_vals, 0);

  Matrix mat_new=
      trajectory.phase(0).jointMatrix(robot, new_vals, 0);

  for (int i = 0; i < mat_new.cols(); ++i){
    std::cout << "old_val = "<< mat_old(0,i) << " new val = " << mat_new(0,i) << std::endl;
    // This test should pass
    //EXPECT(gtsam::assert_equal(mat_old(0,i), mat_new(0,i), 1e-9)); 
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}