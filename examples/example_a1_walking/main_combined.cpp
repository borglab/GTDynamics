/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_combined.cpp
 * @brief Unitree A1 trajectory optimization with pre-specified footholds.
 * @author: Dan Barladeanu
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/dynamics/ChainDynamicsGraph.h>
#include <gtdynamics/utils/ChainInitializer.h>
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
  FootContactVector states = {stationary, RLFR, stationary, RRFL};
  std::vector<size_t> phase_lengths = {2, 10, 2, 10};

  WalkCycle walk_cycle(states, phase_lengths);

  return Trajectory(walk_cycle, repeat);
}

int CombinedRun(bool add_mass_to_body){


  //gttic_(start1); //need to build gtsam with timing for this

  // Load Unitree A1 robot urdf.
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

    // Load Unitree A1 robot urdf.
  auto robot_massless =
      CreateRobotFromFile(kUrdfPath + std::string("/a1/a1.urdf"), "a1");

    // set masses and inertias
  double total_mass = 0.0;
  for (auto&& link : robot_massless.links()) {
    if (link->name().find("trunk") == std::string::npos) {
      link->setMass(0.0);
      link->setInertia(gtsam::Matrix3::Zero());
    }
  }
  
  if (add_mass_to_body) {
    robot_massless.link("trunk")->setMass(12.458);
    std::cout << "robot body mass set to " << robot_massless.link("trunk")->mass() << std::endl;
  }

  double sigma_dynamics =5e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-3;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  gtsam::Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  OptimizerSetting opt(sigma_dynamics, sigma_dynamics, sigma_dynamics, sigma_dynamics);
  opt.p_cost_model = gtsam::noiseModel::Constrained::All(6);
  //opt.q_col_cost_model = Isotropic::Sigma(1, 1e-2);
  //opt.v_col_cost_model = Isotropic::Sigma(1, 1e-2);
  //opt.cm_cost_model = gtsam::noiseModel::Constrained::All(3);
  //opt.f_cost_model = gtsam::noiseModel::Constrained::All(6);
  //opt.fa_cost_model = gtsam::noiseModel::Constrained::All(6);
  //opt.t_cost_model = gtsam::noiseModel::Constrained::All(1);
  //OptimizerSetting opt(1e-3, 1e-3, 1e-3, 1e-2);
  DynamicsGraph graph_mass_builder(opt, gravity);
  DynamicsGraph graph_massless_builder(opt,gravity);
  ChainDynamicsGraph chain_graph_massless_builder(robot_massless, opt, gravity);

  // Create the trajectory, 1 walk cycle.
  auto trajectory = getTrajectory(robot, 3);

  // Create multi-phase trajectory factor graph
  auto collocation = CollocationScheme::Euler;
  std::cout<<"graph_DG_mass"<<std::endl;
  auto graph_DG_mass =
      trajectory.multiPhaseFactorGraph(robot, graph_mass_builder, collocation, mu);
  std::cout<<"graph_DG_massless"<<std::endl;
  auto graph_DG_massless =
      trajectory.multiPhaseFactorGraph(robot_massless, graph_massless_builder, collocation, mu);
  std::cout<<"graph_CDG_massless"<<std::endl;
  auto graph_CDG_massless =
      trajectory.multiPhaseFactorGraph(robot_massless, chain_graph_massless_builder, collocation, mu);

  // Build the objective factors.
  double ground_height = -0.4;
  const Point3 step(0.18, 0.0, 0);
  gtsam::NonlinearFactorGraph objectives =
      trajectory.contactPointObjectives(robot, Isotropic::Sigma(3,8e-6), step, ground_height);

    // Get final time step.
  int K = trajectory.getEndTimeStep(trajectory.numPhases() - 1);

  // Add body goal objectives to the factor graph.
  auto base_link = robot.link("trunk");
  std::cout << "base link mass = " << base_link->mass() << std::endl; 
  for (int k = 0; k <= K; k++) {
    objectives.add(
        LinkObjectives(base_link->id(), k)
            .pose(base_link->bMcom(), Isotropic::Sigma(6, 5e-5))
            .twist(gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5))
            .twistAccel(gtsam::Z_6x1, Isotropic::Sigma(6, 5e-5)));
  }

  // Add prior on A1 lower joint angles
  auto prior_model_lower= Isotropic::Sigma(1, 5e-5);
  auto prior_model_upper = Isotropic::Sigma(1, 5e-5);
  auto prior_model_hip = Isotropic::Sigma(1, 5e-5);
  auto prior_model_va = Isotropic::Sigma(1, 1e-1);
  for (auto&& joint : robot.joints())
    if (joint->name().find("lower") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("lower") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(-1.4, prior_model_lower));
        //.velocity(0.0, prior_model_va).acceleration(0.0, prior_model_va));
      }

  // Add prior on A1 hip joint angles
  //prior_model = Isotropic::Sigma(1, 1e-3);
  for (auto&& joint : robot.joints())
    if (joint->name().find("hip") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("hip") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(0.0, prior_model_hip));
        //.velocity(0.0, prior_model_va).acceleration(0.0, prior_model_va));
      }
 
   // Add prior on A1 hip joint angles
  //prior_model = Isotropic::Sigma(1, 1e-3);
  for (auto&& joint : robot.joints())
    if (joint->name().find("upper") < 100)
      for (int k = 0; k <= K; k++) {
        //std::cout << joint->name() << joint->name().find("hip") << std::endl;
        objectives.add(JointObjectives(joint->id(), k).angle(0.9, prior_model_upper));
        //.velocity(0.0, prior_model_va).acceleration(0.0, prior_model_va));
      }

    // Constrain all Phase keys to have duration of 1 /20.
  const double desired_dt = 1. / 24;
  trajectory.addIntegrationTimeFactors(&objectives, desired_dt, 1e-30);

  // Add objectives to factor graph.
   graph_DG_mass.add(objectives);
   graph_DG_massless.add(objectives);
   graph_CDG_massless.add(objectives);

  gtsam::NonlinearFactorGraph boundary_objectives_DG;
  gtsam::NonlinearFactorGraph boundary_objectives_CDG;

  // Add link and joint boundary conditions to FG.
  trajectory.addBoundaryConditions(&boundary_objectives_DG, robot, dynamics_model_6,
                                   dynamics_model_6, objectives_model_6,
                                  objectives_model_1, objectives_model_1);

  // boundary conditions, using this instead addBoundaryConditions because we dont have link twist and accel variables
  for (auto &&link : robot_massless.links()) {
    // Initial link poses should be bMcom
    const int i = link->id();
    if (i==0)
      boundary_objectives_CDG.add(LinkObjectives(i, 0)
                   .pose(link->bMcom(), dynamics_model_6)
                   .twist(gtsam::Z_6x1, dynamics_model_6));
    if (i==3 || i==6 || i==9 || i==12)
      boundary_objectives_CDG.add(LinkObjectives(i, 0)
                   .pose(link->bMcom(), dynamics_model_6));
    
  }
  // initial and end joint velocity and accelerations should be zero
  boundary_objectives_CDG.add(JointsAtRestObjectives(robot, objectives_model_1,
                                  objectives_model_1, 0));
  boundary_objectives_CDG.add(JointsAtRestObjectives(robot, objectives_model_1,
                                  objectives_model_1, K));

    // Add objectives to factor graph.
   graph_DG_mass.add(boundary_objectives_DG);
   graph_DG_massless.add(boundary_objectives_DG);
   graph_CDG_massless.add(boundary_objectives_CDG);

  // Initialize solution.
  double gaussian_noise = 1e-20;
 Initializer initializer;
 ChainInitializer chain_initializer;
  gtsam::Values init_vals_DG =
      trajectory.multiPhaseInitialValues(robot, initializer, gaussian_noise, desired_dt);
  gtsam::Values init_vals_CDG =
      trajectory.multiPhaseInitialValues(robot_massless, chain_initializer, gaussian_noise, desired_dt);

    // Optimize!
  gtsam::LevenbergMarquardtParams params;
  //params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e10);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  params.setAbsoluteErrorTol(1.0);

  gttic_(optimization1);

  std::cout << "graph_DG_mass size = " << graph_DG_mass.size() << std::endl;
  std::cout << "graph_DG_mass keys size = " << graph_DG_mass.keys().size() << std::endl;
  std::cout << "graph_DG_mass init vals size = " << init_vals_DG.size() << std::endl;
  
  // Optimize!
  gtsam::LevenbergMarquardtOptimizer optimizer_DG_mass(graph_DG_mass, init_vals_DG, params);
  auto results_DG_mass = optimizer_DG_mass.optimize();

  gttoc_(optimization1);
  //gttoc_(start1);

  //gtsam::tictoc_print_(); //need to build gtsam with timing for this
  // Write results to traj file
  trajectory.writeToFile(robot, "a1_traj_DG_mass.csv", results_DG_mass);

 gttic_(optimization2);

  std::cout << "graph_DG_massless size = " << graph_DG_massless.size() << std::endl;
  std::cout << "graph_DG_massless keys size = " << graph_DG_massless.keys().size() << std::endl;
  std::cout << "graph_DG_massless init vals size = " << init_vals_DG.size() << std::endl;
  
  // Optimize!
  gtsam::LevenbergMarquardtOptimizer optimizer_DG_massless(graph_DG_massless, init_vals_DG, params);
  auto results_DG_massless = optimizer_DG_massless.optimize();

  gttoc_(optimization2);

  //gtsam::tictoc_print_(); //need to build gtsam with timing for this
  // Write results to traj file
  trajectory.writeToFile(robot_massless, "a1_traj_DG_massless.csv", results_DG_massless);
  
   gttic_(optimization3);

  std::cout << "graph_CDG_massless size = " << graph_CDG_massless.size() << std::endl;
  std::cout << "graph_CDG_massless keys size = " << graph_CDG_massless.keys().size() << std::endl;
  std::cout << "graph_CDG_massless init vals size = " << init_vals_CDG.size() << std::endl;
  
  // Optimize!
  gtsam::LevenbergMarquardtOptimizer optimizer_CDG_massless(graph_CDG_massless, init_vals_CDG, params);
  auto results_CDG_massless = optimizer_CDG_massless.optimize();

  gttoc_(optimization3);

  //gtsam::tictoc_print_(); //need to build gtsam with timing for this
  // Write results to traj file
  trajectory.writeToFile(robot_massless, "a1_traj_CDG_massless.csv", results_CDG_massless);

  gtsam::tictoc_print_(); 

  std::ofstream FileLocations("locations.txt");
  std::ofstream FileTorques;
  std::ofstream FileWrenchesBody;
  std::ofstream FileWrenchesGround;
  if (add_mass_to_body){
    FileWrenchesBody.open("body_wrenches_body_full_mass.txt");
    FileWrenchesGround.open("ground_wrenches_body_full_mass.txt");
    FileTorques.open("torques_body_full_mass.txt");
  }
  else {
    FileWrenchesBody.open("body_wrenches_body_only_mass.txt");
    FileWrenchesGround.open("ground_wrenches_body_only_mass.txt");
    FileTorques.open("torques_body_only_mass.txt");
  }

  const int link_to_test = 0;
  const int joint_to_test = 0;
  for (int i = 0; i < K;++i){
    //std::cout << results_DG_massless.at<gtsam::Vector6>(WrenchKey(0,0,i)) << std::endl;
    //std::cout << results_CDG_massless.at<gtsam::Vector6>(WrenchKey(0,0,i)) << std::endl;
    gtsam::Pose3 pose1 = results_DG_mass.at<gtsam::Pose3>(PoseKey(link_to_test,i));
    gtsam::Pose3 pose2 =  results_DG_massless.at<gtsam::Pose3>(PoseKey(link_to_test,i));
    gtsam::Pose3 pose3 = results_CDG_massless.at<gtsam::Pose3>(PoseKey(link_to_test,i)) ;
    FileLocations << pose1.translation().x() << "," << pose1.translation().y() <<","<< pose1.translation().z() <<","<<
                        pose2.translation().x() << "," << pose2.translation().y() << ","<<pose2.translation().z() <<","<<
                        pose3.translation().x() << "," << pose3.translation().y() << ","<<pose3.translation().z() <<"\n";
    gtsam::Vector6 wrench1 = results_DG_mass.at<gtsam::Vector6>(WrenchKey(0,joint_to_test,i));
    gtsam::Vector6 wrench2 = results_DG_massless.at<gtsam::Vector6>(WrenchKey(0,joint_to_test,i));
    gtsam::Vector6 wrench3= results_CDG_massless.at<gtsam::Vector6>(WrenchKey(0,joint_to_test,i));
    FileWrenchesBody << wrench1[0] << "," << wrench1[1]  <<","<< wrench1[2]  <<","<< wrench1[3]<<","<< wrench1[4]<<","<< wrench1[5]<<","<<
                                      wrench2[0] << "," << wrench2[1]  <<","<< wrench2[2]  <<","<< wrench2[3]<<","<< wrench2[4]<<","<< wrench2[5]<<","<<
                                       wrench3[0] << "," << wrench3[1]  <<","<< wrench3[2]  <<","<< wrench3[3]<<","<< wrench3[4]<<","<< wrench3[5]<<"\n";

    double angle0 = results_CDG_massless.at<double>(JointAngleKey(0,i));
    double angle1 = results_CDG_massless.at<double>(JointAngleKey(1,i));
    double angle2 = results_CDG_massless.at<double>(JointAngleKey(2,i));
    Pose3 T0 = robot.joint("FL_hip_joint")->parentTchild(angle0);
    Pose3 T1 = robot.joint("FL_upper_joint")->parentTchild(angle1);
    Pose3 T2 = robot.joint("FL_lower_joint")->parentTchild(angle2);
    Pose3 T3 = Pose3(Rot3(),Point3(0.0, 0.0, -0.07));
    Pose3 T_body_contact = T0 * T1 * T2 * T3;

    double torque0_dg_mass = results_DG_mass.at<double>(TorqueKey(0,i));
    double torque1_dg_mass = results_DG_mass.at<double>(TorqueKey(1,i));
    double torque2_dg_mass = results_DG_mass.at<double>(TorqueKey(2,i));
    
    double torque0_dg_massless = results_DG_mass.at<double>(TorqueKey(0,i));
    double torque1_dg_massless = results_DG_mass.at<double>(TorqueKey(1,i));
    double torque2_dg_massless = results_DG_mass.at<double>(TorqueKey(2,i));

    gtsam::Vector6 wrench6= results_CDG_massless.at<gtsam::Vector6>(WrenchKey(0,0,i));

    double torque0_cdg_massless = (-1) * wrench6.transpose() * T0.AdjointMap() * robot.joint("FL_hip_joint")->cScrewAxis();
    double torque1_cdg_massless = (-1) * wrench6.transpose() * (T0*T1).AdjointMap()* robot.joint("FL_upper_joint")->cScrewAxis();
    double torque2_cdg_massless = (-1) * wrench6.transpose() * (T0*T1*T2).AdjointMap()* robot.joint("FL_lower_joint")->cScrewAxis();
    
    FileTorques<<  torque0_dg_mass <<',' << torque1_dg_mass << ',' << torque2_dg_mass << ','<<
                                    torque0_dg_massless <<',' << torque1_dg_massless << ',' << torque2_dg_massless << ','<<
                                    torque0_cdg_massless <<',' << torque1_cdg_massless << ',' << torque2_cdg_massless << '\n';

    
    if (i <13 || i >25) continue;

    gtsam::Vector6 wrench4 = results_DG_mass.at<gtsam::Vector6>(ContactWrenchKey(3,0,i)).transpose() * T3.AdjointMap();
    gtsam::Vector6 wrench5 = results_DG_massless.at<gtsam::Vector6>(ContactWrenchKey(3,0,i)).transpose() * T3.AdjointMap();
    gtsam::Vector6 wrench7 = wrench6.transpose() * T_body_contact.AdjointMap();
    FileWrenchesGround << wrench4[0] << "," << wrench4[1]  <<","<<wrench4[2]  <<","<< wrench4[3]<<","<< wrench4[4]<<","<< wrench4[5]<<","<<
                                      wrench5[0] << "," << wrench5[1]  <<","<<wrench5[2]  <<","<< wrench5[3]<<","<< wrench5[4]<<","<< wrench5[5]<<","<<
                                       wrench7[0] << "," << wrench7[1]  <<","<< wrench7[2]  <<","<< wrench7[3]<<","<< wrench7[4]<<","<< wrench7[5]<<"\n";
  }
  FileLocations.close();
  FileWrenchesBody.close();
  FileWrenchesGround.close();
  return 0;
}

int main(int argc, char** argv) {
  int a = CombinedRun(false);
  int b = CombinedRun(true);
  return 0;
}