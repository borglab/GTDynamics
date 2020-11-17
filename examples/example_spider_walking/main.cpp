/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Spider trajectory optimization with pre-specified footholds.
 * @Author: Alejandro Escontrela
 * @Author: Stephanie McCormick
 * @Author: Disha Das
 * @Author: Tarushree Gandhi
 */

#include <gtdynamics/utils/TrajectoryParams.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <algorithm>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#define GROUND_HEIGHT -1.75 //-1.75

using gtdynamics::PoseKey, gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey, gtsam::Point3,
    gtsam::Rot3, gtsam::Pose3, gtsam::Values,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string, gtsam::noiseModel::Isotropic, gtdynamics::Phase;
using namespace gtsam;

//Returns a Trajectory object for a single spider walk cycle.
gtdynamics::TrajectoryParams getSpiderTrajParams(vector<string> links, std::string walk_cycle_name, Robot robot)
{
    //Create Trajectory object
    gtdynamics::TrajectoryParams traj_params;
    traj_params.addLinks(links);
    traj_params.generateContactPoints(gtsam::Point3(0, 0.19, 0), GROUND_HEIGHT);
    vector<pair<string, int>> walk_cycle; 

    if (walk_cycle_name == "motion_in_sequence"){
        walk_cycle.push_back(pair<string, int>("stationary", 20));
        walk_cycle.push_back(pair<string, int>("l1", 20));
        walk_cycle.push_back(pair<string, int>("l2", 20));
        walk_cycle.push_back(pair<string, int>("l3", 20));
        walk_cycle.push_back(pair<string, int>("l4", 20));
        walk_cycle.push_back(pair<string, int>("l5", 20));
        walk_cycle.push_back(pair<string, int>("l6", 20));
        walk_cycle.push_back(pair<string, int>("l7", 20));
        walk_cycle.push_back(pair<string, int>("l8", 20));
        traj_params.setWalkCycle(walk_cycle);
        traj_params.addStance("stationary", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l1", robot, {"tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l2", robot, {"tarsus_1", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l3", robot, {"tarsus_1", "tarsus_2", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l4", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l5", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l6", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_7", "tarsus_8"});
        traj_params.addStance("l7", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_8"});
        traj_params.addStance("l8", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7"});
        return traj_params;
    }
    if (walk_cycle_name == "alternating_tetrapod"){
        walk_cycle.push_back(pair<string, int>("stationary", 40));
        walk_cycle.push_back(pair<string, int>("even_legs", 20));
        walk_cycle.push_back(pair<string, int>("stationary", 40));
        walk_cycle.push_back(pair<string, int>("odd_legs", 20));
        traj_params.setWalkCycle(walk_cycle);
        traj_params.addStance("stationary", robot, {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"});
        traj_params.addStance("even_legs",robot, {"tarsus_2", "tarsus_4", "tarsus_6", "tarsus_8"});
        traj_params.addStance("odd_legs", robot, {"tarsus_1", "tarsus_3", "tarsus_5", "tarsus_7"});
        return traj_params;
    }
}

int main(int argc, char **argv)
{
    // Load Stephanie's spider robot.
    auto spider = gtdynamics::CreateRobotFromFile("../spider_alt.sdf", "spider");
    spider.printRobot();

    double sigma_dynamics = 1e-5;   // std of dynamics constraints.
    double sigma_objectives = 1e-6; // std of additional objectives.
    double sigma_joints = 1.85e-4;  //1.85e-4

    // Noise models.
    auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
         dynamics_model_3 = Isotropic::Sigma(3, sigma_dynamics),
         dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
         dynamics_model_1_2 = Isotropic::Sigma(1, sigma_joints),
         objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
         objectives_model_3 = Isotropic::Sigma(3, sigma_objectives),
         objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

    // TODO(aescontrela): Make a constructor for OptimizerSetting that
    //     initializes all noise models with the same sigma.
    auto opt = gtdynamics::OptimizerSetting();
    opt.bp_cost_model = dynamics_model_6;
    opt.bv_cost_model = dynamics_model_6;
    opt.ba_cost_model = dynamics_model_6;
    opt.p_cost_model = dynamics_model_6;
    opt.v_cost_model = dynamics_model_6;
    opt.a_cost_model = dynamics_model_6;
    opt.f_cost_model = dynamics_model_6;
    opt.fa_cost_model = dynamics_model_6;
    opt.t_cost_model = dynamics_model_1;
    opt.cp_cost_model = dynamics_model_1;
    opt.cfriction_cost_model = dynamics_model_1;
    opt.cv_cost_model = dynamics_model_3;
    opt.ca_cost_model = dynamics_model_3;
    opt.planar_cost_model = dynamics_model_3;
    opt.prior_q_cost_model = dynamics_model_1;
    opt.prior_qv_cost_model = dynamics_model_1;
    opt.prior_qa_cost_model = dynamics_model_1;
    opt.prior_t_cost_model = dynamics_model_1;
    opt.q_col_cost_model = dynamics_model_1;
    opt.v_col_cost_model = dynamics_model_1;
    opt.time_cost_model = dynamics_model_1;
    auto graph_builder = gtdynamics::DynamicsGraph(opt);

    // Env parameters.
    Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
    double mu = 1.0;

    // All contacts.
    typedef ContactPoints CPs;
    string walk_cycle_name_1 = "alternating_tetrapod";
    string walk_cycle_name_2 = "motion_in_sequence";
    vector<string> links = {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"};

    gtdynamics::TrajectoryParams traj_params = getSpiderTrajParams(links, walk_cycle_name_1, spider);

    int repeat = 2;
    auto spider_trajectory = gtdynamics::Trajectory(traj_params, repeat);

    //Get phase information
    vector<CPs> phase_cps = spider_trajectory.phaseCPs();
    vector<int> phase_durations = spider_trajectory.phaseDurations();
    vector<Robot> robots = spider_trajectory.phaseRobotModels();

    // Define noise to be added to initial values, desired timestep duration,
    // vector of link name strings, robot model for each phase, and
    // phase transition initial values.
    double gaussian_noise = 1e-5;
    double dt_des = 1. / 240;
    vector<Values> transition_graph_init = spider_trajectory.getInitTransitionValues(gaussian_noise);

    // Get final time step.
    int t_f = spider_trajectory.getEndTimeStep(spider_trajectory.numPhases() - 1); // Final timestep.

    // Collocation scheme.
    auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

    // Graphs for transition between phases + their initial values.
    vector<gtsam::NonlinearFactorGraph> transition_graphs = spider_trajectory.getTransitionGraphs(graph_builder, gravity, mu);
    
    // Construct the multi-phase trajectory factor graph.
    //TODO: Pass Trajectory here
    std::cout << "Creating dynamics graph" << std::endl;
    auto graph = graph_builder.multiPhaseTrajectoryFG(
        robots, phase_durations, transition_graphs, collocation, gravity, boost::none, phase_cps, mu);

    // Build the objective factors.
    gtsam::NonlinearFactorGraph objective_factors;
    auto base_link = spider.getLinkByName("body");

    std::map<string, gtdynamics::LinkSharedPtr> link_map;
    for (auto &&link : links)
        link_map.insert(std::make_pair(link, spider.getLinkByName(link)));

    // Previous contact point goal.
    std::map<string, Point3> prev_cp = spider_trajectory.initContactPointGoal();

    // Distance to move contact point per time step during swing.
    auto contact_offset = Point3(0, 0.02, 0);

    // Add contact point objectives to factor graph.
    for (int p = 0; p < spider_trajectory.numPhases(); p++)
    {
        // Phase start and end timesteps.
        int t_p_i = spider_trajectory.getStartTimeStep(p);
        int t_p_f = spider_trajectory.getEndTimeStep(p);

        // Obtain the contact links and swing links for this phase.
        vector<string> phase_contact_links = spider_trajectory.getPhaseContactLinks(p);
        vector<string> phase_swing_links = spider_trajectory.getPhaseSwingLinks(p);

        for (int t = t_p_i; t <= t_p_f; t++)
        {
            cout << prev_cp["tarsus_8"] << endl;
            // Normalized phase progress.
            double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

            for (auto &&pcl : phase_contact_links)
                objective_factors.add(spider_trajectory.pointGoalFactor(
                                            pcl, t, Isotropic::Sigma(3, 1e-7), //1e-7
                                            Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.05)));
                                            
            double h = GROUND_HEIGHT + std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

            for (auto &&psl : phase_swing_links)
                objective_factors.add(spider_trajectory.pointGoalFactor(
                                            psl, t, Isotropic::Sigma(3, 1e-7), 
                                            Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));

            // Update the goal point for the swing links.
            for (auto &&psl : phase_swing_links)
                prev_cp[psl] = prev_cp[psl] + contact_offset;
        }
    }

    // Add base goal objectives to the factor graph.
    for (int t = 0; t <= t_f; t++){
        objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            PoseKey(base_link->getID(), t),
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.0, 0.5)), //0.5
            Isotropic::Sigma(6, 5e-5)));                             //6.2e-5 //5e-5
        objective_factors.add(gtsam::PriorFactor<Vector6>(
            TwistKey(base_link->getID(), t), Vector6::Zero(), Isotropic::Sigma(6, 5e-5)));
    }

    // Add link boundary conditions to FG.
    for (auto &&link : spider.links()){
        // Initial link pose, twists.
        objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            PoseKey(link->getID(), 0), link->wTcom(), dynamics_model_6));
        objective_factors.add(gtsam::PriorFactor<Vector6>(
            TwistKey(link->getID(), 0), Vector6::Zero(), dynamics_model_6));

        // Final link twists, accelerations.
        objective_factors.add(gtsam::PriorFactor<Vector6>(
            TwistKey(link->getID(), t_f), Vector6::Zero(), objectives_model_6));
        objective_factors.add(gtsam::PriorFactor<Vector6>(
            TwistAccelKey(link->getID(), t_f), Vector6::Zero(),
            objectives_model_6));
    }

    // Add joint boundary conditions to FG.
    for (auto &&joint : spider.joints()){
        //Add priors to joint angles
        for (int t = 0; t <= t_f; t++){
            if (joint->name().find("hip2") == 0)
                objective_factors.add(gtsam::PriorFactor<double>(JointAngleKey(joint->getID(), t), 2.5, dynamics_model_1_2));
        }
        objective_factors.add(gtsam::PriorFactor<double>(
            JointVelKey(joint->getID(), 0), 0.0, dynamics_model_1));
        objective_factors.add(gtsam::PriorFactor<double>(
            JointVelKey(joint->getID(), t_f), 0.0, objectives_model_1));
        objective_factors.add(gtsam::PriorFactor<double>(
            JointAccelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    }
    //TODO: Create const Trajectory method called walk_cycleSize/numPhases
    // Add prior factor constraining all Phase keys to have duration of 1 / 240.
    for (int phase = 0; phase < spider_trajectory.numPhases(); phase++)
        objective_factors.add(gtsam::PriorFactor<double>(
            PhaseKey(phase), dt_des,
            gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

    // Add min torque objectives.
    for (int t = 0; t <= t_f; t++)
    {
        for (auto &&joint : spider.joints())
            objective_factors.add(gtdynamics::MinTorqueFactor(
                TorqueKey(joint->getID(), t),
                gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
    }
    graph.add(objective_factors);

    //TODO: Pass Trajectory here
    // Initialize solution.
    gtsam::Values init_vals;
    init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
        robots, phase_durations, transition_graph_init, dt_des, gaussian_noise,
        phase_cps);
    
    // Optimize!
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.setlambdaInitial(1e0);
    params.setlambdaLowerBound(1e-7);
    params.setlambdaUpperBound(1e10);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
    auto results = optimizer.optimize();

    //Write results to traj file
    vector<string> jnames;
    for (auto &&joint : spider.joints())
        jnames.push_back(joint->name());
    std::cout<<jnames.size() <<std::endl;
    string jnames_str = boost::algorithm::join(jnames, ",");
    std::ofstream traj_file;

    //Get current directory to save the generated traj.csv file
    char cwd[PATH_MAX];
    char *fgh = getcwd(cwd, PATH_MAX);
    string example_directory = strcat(cwd, "/..");

    traj_file.open(example_directory + "/forward_traj.csv");
    // angles, vels, accels, torques, time.
    traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
              << jnames_str << ",t"
              << "\n";
    for (int phase = 0; phase < spider_trajectory.numPhases(); phase++)
        spider_trajectory.writePhaseToFile(traj_file, results, phase);

    //Write the last 4 phases to disk n times
    for (int i = 0; i < 100; i++)
    {
        for (int phase = 4; phase < phase_durations.size(); phase++)
            spider_trajectory.writePhaseToFile(traj_file, results, phase);
    }
    traj_file.close();
    return 0;
}
