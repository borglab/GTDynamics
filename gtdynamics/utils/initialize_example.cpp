#include "gtdynamics/utils/initialize_example.h"

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/factors/MinTorqueFactor.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/initialize_solution_utils.h"
#include "gtdynamics/factors/PointGoalFactor.h"

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
#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

// TODO (disha + stephanie): move to GTDynamics/examples/utils?
// (may be more convenient to do so if examples' compilation process
// remains separate from the rest of GTDynamics')

namespace gtdynamics {

using gtdynamics::PoseKey, gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey, gtsam::Point3,
    gtsam::Rot3, gtsam::Pose3, gtsam::Values,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string, gtsam::noiseModel::Isotropic;

typedef ContactPoints CPs;

gtdynamics::DynamicsGraph InitializeGraphBuilder(boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_3,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_1,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_3,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_1){
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

  return gtdynamics::DynamicsGraph(opt);
}

// TODO (disha + stephanie): may want to split this function up
// (smaller, static helper functions? see sdf.h / sdf.cpp for reference)
gtsam::NonlinearFactorGraph CreateFactorGraph(Vector3 gravity, double mu,
                double ground_height, double sigma_objectives,
                boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_6,
                boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_1,
                boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_6,
                boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_3,
                boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_1,
                gtdynamics::DynamicsGraph graph_builder,
                vector<Values> &transition_graph_init,
                gtdynamics::Robot robot, vector<string> links,
                double dt_des, double gaussian_noise,
                vector<Robot> robots, vector<CPs> phase_cps,
                vector<CPs> trans_cps, vector<int> phase_steps,
                gtsam::Point3 contact_point) {
    // Define the cumulative phase steps.
    vector<int> cum_phase_steps;
    for (int i = 0; i < phase_steps.size(); i++) {
        int cum_val = i == 0 ? phase_steps[0] : phase_steps[i] + cum_phase_steps[i-1];
        cum_phase_steps.push_back(cum_val);
        std::cout << cum_val << std::endl;
    }
    int t_f = cum_phase_steps[cum_phase_steps.size() - 1];  // Final timestep.

    // Collocation scheme.
    auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

    // Graphs for transition between phases + their initial values.
    vector<gtsam::NonlinearFactorGraph> transition_graphs;
    for (int p = 1; p < phase_cps.size(); p++) {
        std::cout << "Creating transition graph" << std::endl;
        transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        robots[p], cum_phase_steps[p - 1], gravity, boost::none, trans_cps[p - 1], mu));
        std::cout << "Creating initial values" << std::endl;
        transition_graph_init.push_back(
        ZeroValues(robots[p], cum_phase_steps[p - 1], gaussian_noise, trans_cps[p - 1]));
    }

    // Construct the multi-phase trajectory factor graph.
    std::cout << "Creating dynamics graph" << std::endl;
    auto graph = graph_builder.multiPhaseTrajectoryFG(
        robots, phase_steps, transition_graphs, collocation, gravity, boost::none,
        phase_cps, mu);

    // Build the objective factors.
    gtsam::NonlinearFactorGraph objective_factors;
    auto base_link = robot.getLinkByName("body");

    std::map<string, gtdynamics::LinkSharedPtr> link_map;
    for (auto&& link : links)
        link_map.insert(std::make_pair(link, robot.getLinkByName(link)));

    // Previous contact point goal.
    std::map<string, Point3> prev_cp;
    for (auto&& link : links)
        prev_cp.insert(std::make_pair(link,
        (link_map[link]->wTcom() * Pose3(Rot3(), contact_point)).translation()));
        
    // Distance to move contact point during swing.
    auto contact_offset = Point3(0.15, 0, 0);
        
    // Add contact point objectives to factor graph.
    for (int p = 0; p < phase_cps.size(); p++) {
        // Phase start and end timesteps.
        int t_p_i = cum_phase_steps[p] - phase_steps[p];
        if (p != 0) t_p_i += 1;
        int t_p_f = cum_phase_steps[p];

        // Obtain the contact links and swing links for this phase.
        vector<string> phase_contact_links;
        for (auto&& cp : phase_cps[p])
        phase_contact_links.push_back(cp.name);
        vector<string> phase_swing_links;
        for (auto&& l : links) {
            if (std::find(phase_contact_links.begin(),
                    phase_contact_links.end(), l) == phase_contact_links.end())
                phase_swing_links.push_back(l);
        }

        if (p==2)
        contact_offset = 2 * contact_offset;

        // Update the goal point for the swing links.
        for (auto && psl : phase_swing_links)
        prev_cp[psl] = prev_cp[psl] + contact_offset;

        for (int t = t_p_i; t <= t_p_f; t++) {
            // Normalized phase progress.
            double t_normed = (double) (t - t_p_i) / (double) (t_p_f - t_p_i);
            for (auto&& pcl : phase_contact_links)
                // TODO(aescontrela): Use correct contact point for each link.
                objective_factors.add(gtdynamics::PointGoalFactor(
                PoseKey(link_map[pcl]->getID(), t), objectives_model_3,
                Pose3(Rot3(), contact_point), Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), ground_height - 0.03)));

            double h = ground_height + 0.05 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

            for (auto&& psl : phase_swing_links)
                objective_factors.add(gtdynamics::PointGoalFactor(
                PoseKey(link_map[psl]->getID(), t), objectives_model_3,
                Pose3(Rot3(), contact_point), Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));
        }
    }

    // Add base goal objectives to the factor graph.
    auto base_pose_model = gtsam::noiseModel::Diagonal::Sigmas(
        (Vector(6) << sigma_objectives * 3, sigma_objectives * 3,
                    sigma_objectives * 3, 10000, sigma_objectives,
                    sigma_objectives).finished());
    for (int t = 0; t <= t_f; t++)
        objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(base_link->getID(), t),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.13)),
        base_pose_model));

    // Add link boundary conditions to FG.
    for (auto&& link : robot.links()) {
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
    for (auto&& joint : robot.joints()) {
        objective_factors.add(gtsam::PriorFactor<double>(
            JointAngleKey(joint->getID(), 0), 0.0, dynamics_model_1));
        objective_factors.add(gtsam::PriorFactor<double>(
            JointVelKey(joint->getID(), 0), 0.0, dynamics_model_1));

        objective_factors.add(gtsam::PriorFactor<double>(
            JointVelKey(joint->getID(), t_f), 0.0, objectives_model_1));
        objective_factors.add(gtsam::PriorFactor<double>(
            JointAccelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    }

    // Add prior factor constraining all Phase keys to have duration of 1 / 240.
    for (int phase = 0; phase < phase_steps.size(); phase++)
        objective_factors.add(gtsam::PriorFactor<double>(
            PhaseKey(phase), dt_des,
            gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

    // Add min torque objectives.
    for (int t = 0; t <= t_f; t++) {
        for (auto&& joint : robot.joints())
        objective_factors.add(gtdynamics::MinTorqueFactor(
            TorqueKey(joint->getID(), t),
            gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
    }
    graph.add(objective_factors);

    return graph;
}

gtsam::Values Optimize(gtsam::NonlinearFactorGraph graph, 
                         vector<Values> &transition_graph_init,
                         double dt_des, double gaussian_noise, 
                         vector<Robot> robots, vector<CPs> phase_cps,
                         vector<int> phase_steps) {
  // Initialize solution.
  gtsam::Values init_vals;
  init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
    robots, phase_steps, transition_graph_init, dt_des, gaussian_noise,
    phase_cps);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e0);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  return optimizer.optimize();
}

void CreateTrajFile(gtdynamics::Robot robot, 
                    vector<int> phase_steps, gtsam::Values results) {
  vector<string> jnames;
  for (auto&& joint : robot.joints()) jnames.push_back(joint->name());
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;

  //Get current directory to save the generated traj.csv file
  char cwd[PATH_MAX];
  char* fgh = getcwd(cwd, PATH_MAX);
  string example_directory = strcat(cwd ,"/..");

  traj_file.open(example_directory + "/traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  int t = 0;
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {

      vector<string> vals;
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(results.atDouble(JointVelKey(joint->getID(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(results.atDouble(TorqueKey(joint->getID(), t))));
      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));
      t++;
      string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
  traj_file.close();
}

}  // namespace gtdynamics
