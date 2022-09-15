/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/linear/Sampler.h>

#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/PoseFactor.h>

#include "gtdynamics/factors/CollocationFactors.h"
#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/optimizer/ConstrainedOptimizer.h"
#include "gtdynamics/manifold/ConstraintManifold.h"
#include "gtdynamics/manifold/TspaceBasis.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"
#include "gtdynamics/optimizer/OptimizationBenchmark.h"

#include "gtdynamics/bloesch/bloesch2.h"
#include "gtdynamics/bloesch/bloesch3.h"
#include "gtdynamics/bloesch/SimpleEstimator.h"
#include "gtdynamics/bloesch/robot_imu.h"
#include "gtdynamics/bloesch/utils.h"

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace gtdynamics;
using namespace gtsam;
using namespace lrse;

typedef std::map<std::string, int> Contacts;
std::string model_file = "/Users/yetongzhang/packages/lrse/models/a1.urdf";
Robot robot = gtdynamics::CreateRobotFromFile(model_file, "", true);
std::vector<std::string> feet{"FR_toe", "FL_toe", "RR_toe", "RL_toe"};
std::string base_name = "trunk";
size_t base_id = robot.link(base_name)->id();

KeyVector FindBasisKeys(const ConnectedComponent::shared_ptr &cc) {
  KeyVector basis_keys;

  std::set<int> unfixed_joints;
  for (const auto& joint : robot.joints()) {
    if (joint->type() != gtdynamics::Joint::Type::Fixed) {
      unfixed_joints.insert(joint->id());
    }
  }

  for (const Key &key : cc->keys_) {
    auto symb = gtdynamics::DynamicsSymbol(key);
    if (symb.label() == "p" && symb.linkIdx()==base_id) {
      basis_keys.push_back(key);
    }
    else if (symb.label() == "q") {
      if (unfixed_joints.find(symb.jointIdx())!=unfixed_joints.end()) {
        basis_keys.push_back(key);
      }
    }
  }

  return basis_keys;
}


void ReadData(const std::string file_path, std::vector<double> &timestamps,
              std::vector<Vector3> &omegas, std::vector<Vector3> &accs,
              std::vector<Values> &joint_angles_vec,
              std::vector<Contacts> &contacts_vec) {

  auto data = ReadCsv(file_path);

  for (size_t k=0; k<data.size(); k++) {
    const auto& data_k = data.at(k);
    if (data_k.size() == 0) {
      continue;
    }
    timestamps.push_back(std::stod(data_k[0]));
    omegas.push_back(Vector3(std::stod(data_k[1]), std::stod(data_k[2]), std::stod(data_k[3])));
    accs.push_back(Vector3(std::stod(data_k[4]), std::stod(data_k[5]), std::stod(data_k[6])));
    Contacts contacts;
    for (size_t ft_idx=0; ft_idx<4; ft_idx++) {
      if (data_k[ft_idx+7] == "True") {
        contacts[feet[ft_idx]] = 1;
      }
      else if (data_k[ft_idx+7] == "False") {
        contacts[feet[ft_idx]] = 0;
      }
      else {
        throw std::runtime_error("neither True or False.");
      }
    }
    contacts_vec.push_back(contacts);
    Values joint_angles;
    for (size_t joint_idx=0; joint_idx<robot.numJoints(); joint_idx++) {
      double q = std::stod(data_k[11+joint_idx]);
      InsertJointAngle(&joint_angles, joint_idx, k, q);
    }
    joint_angles_vec.push_back(joint_angles);
  }
}


void ReadGTData(const std::string file_path, std::vector<NavState> &states) {
  auto data = ReadCsv(file_path);
  for (size_t k=0; k<data.size(); k++) {
    const auto& data_k = data.at(k);
    if (data_k.size() == 0) {
      continue;
    }
    Rot3 R = Rot3::Quaternion(std::stod(data_k[0]), std::stod(data_k[1]), std::stod(data_k[2]), std::stod(data_k[3]));
    Vector3 t(std::stod(data_k[4]), std::stod(data_k[5]), std::stod(data_k[6]));
    Vector3 v(std::stod(data_k[7]), std::stod(data_k[8]), std::stod(data_k[9]));
    states.push_back(NavState(R, t, v));
  }
}

Values GetJointAngles(const Values& values, size_t k, boost::optional<Sampler&> sampler = boost::none) {
  Values joint_angles;
  for (const auto& joint: robot.joints()) {
    double q = JointAngle(values, joint->id(), k);
    if (sampler) {
      q = q + sampler->sample()(0);
    }
    InsertJointAngle(&joint_angles, joint->id(), 0, q);
  }
  return joint_angles;
}

Pose3 AddNoiseToPose(const Pose3& pose, Sampler& sampler) {
  auto xi = sampler.sample();
  return pose.expmap(xi);
}


void TrajectoryOptimization() {
  /// Initialize

  std::map<std::string, gtsam::Point3> contact_in_com;
  contact_in_com["FR_toe"] = Point3(0, 0, 0);
  contact_in_com["FL_toe"] = Point3(0, 0, 0);
  contact_in_com["RR_toe"] = Point3(0, 0, 0);
  contact_in_com["RL_toe"] = Point3(0, 0, 0);

  std::vector<double> timestamps;
  std::vector<Vector3> omegas;
  std::vector<Vector3> accs;
  std::vector<Values> joint_angles_vec;
  std::vector<Contacts> contacts_vec;
  ReadData("/Users/yetongzhang/packages/lrse/data.csv", timestamps, omegas, accs, joint_angles_vec, contacts_vec);

  std::vector<NavState> states;
  ReadGTData("/Users/yetongzhang/packages/lrse/gt_data.csv", states);

  // create ground-truth
  Values gt_values;
  for (size_t k=0; k<states.size(); k++) {
    Pose3 base_pose = states[k].pose();
    Values known_values = joint_angles_vec[k];
    InsertPose(&known_values, base_id, k, base_pose);
    Values fk_values = robot.forwardKinematics(known_values, k, base_name);
    for (const auto& joint: robot.joints()) {
      InsertJointAngle(&gt_values, joint->id(), k, JointAngle(fk_values, joint->id(), k));
    }
    for (const auto& link: robot.links()) {
      InsertPose(&gt_values, link->id(), k, Pose(fk_values, link->id(), k));
    }
    for (const auto& foot: feet) {
      auto link = robot.link(foot);
      Key point_key = LinkContactPointKey(link->id(), k);
      Point3 wPcontact = PointOnLink(link, contact_in_com[foot]).predict(gt_values, k);
      gt_values.insert(point_key, wPcontact);
    }
  }

  // calibrate
  std::vector<Point3> point_diff;
  for (const auto& foot: feet) {
    for (size_t k=1; k<states.size(); k++) {
      auto link = robot.link(foot);
      if (contacts_vec[k].at(foot) && contacts_vec[k-1].at(foot)) {
        Key point_key_curr = LinkContactPointKey(link->id(), k);
        Key point_key_prev = LinkContactPointKey(link->id(), k -1);
        Point3 wPcontact_curr = gt_values.at<Point3>(point_key_curr);
        Point3 wPcontact_prev = gt_values.at<Point3>(point_key_prev);
        point_diff.push_back(wPcontact_curr - wPcontact_prev);
      }
    }
  }
  Point3 avg_diff(0, 0, 0);
  for (const auto& p_diff: point_diff) {
    avg_diff += p_diff;
  }
  avg_diff =avg_diff/point_diff.size();
  std::cout << "avg_diff: " << avg_diff.transpose() << "\n";

  double var_x=0, var_y=0, var_z = 0;
  for (const auto& p_diff: point_diff) {
    Point3 var = p_diff - avg_diff;
    var_x += pow(var.x(), 2);
    var_y += pow(var.y(), 2);
    var_z += pow(var.z(), 2);
  }
  double sigma_x = sqrt(var_x/point_diff.size());
  double sigma_y = sqrt(var_y/point_diff.size());
  double sigma_z = sqrt(var_z/point_diff.size());
  std::cout << "sigma: " << sigma_x <<", " << sigma_y << ", " << sigma_z << "\n";
  Vector3 contact_point_sigmas(sigma_x, sigma_y, sigma_z);

  
  // Create measurements
  SimpleEstimator estimator;
  double joint_sigma = M_PI/180.0;
  double r_sigma = 1e-2;
  double t_sigma = 1e-2;
  Vector6 pose_sigmas = (Vector(6) << r_sigma, r_sigma, r_sigma, t_sigma, t_sigma, t_sigma).finished();
  estimator.prior_pose_model_ = noiseModel::Isotropic::Sigma(6, 1e-3);
  estimator.rel_pose_model_ = noiseModel::Diagonal::Sigmas(pose_sigmas);
  estimator.joint_measurement_model_ = noiseModel::Isotropic::Sigma(1, joint_sigma);
  estimator.contact_point_model_ = noiseModel::Diagonal::Sigmas(contact_point_sigmas);
  estimator.fixed_joint_model_ = noiseModel::Isotropic::Sigma(1, 1e-1);
  estimator.kinematics_constraint_model_ = noiseModel::Isotropic::Sigma(6, 1e-1);
  estimator.fixed_point_model_ = noiseModel::Isotropic::Sigma(3, 1e-1);
  estimator.avg_contact_diff_ = Point3(0, 0, 0);
  // estimator.avg_contact_diff_ = Point3(avg_diff.x(), 0, 0);
  double constraint_unit_scale =1e-1;

  // add initial condition
  estimator.setInitial(Pose(gt_values, base_id, 0), GetJointAngles(gt_values, 0), contacts_vec.at(0));

  // steps
  int freq = 5;
  size_t k = freq;
  Sampler pose_sampler(pose_sigmas);
  
  Vector1 sigma_vec(joint_sigma);
  Sampler joint_sampler(sigma_vec);

  while (k < states.size()) {
    // relative pose with noise
    Pose3 prev_pose = Pose(gt_values, base_id, k-freq);
    Pose3 curr_pose = Pose(gt_values, base_id, k);
    Pose3 gt_rel_pose = prev_pose.inverse().compose(curr_pose);
    Pose3 measured_rel_pose = AddNoiseToPose(gt_rel_pose, pose_sampler);

    // joint angle with noise
    auto joint_angles = GetJointAngles(gt_values, k, joint_sampler);

    // contacts
    auto contacts = contacts_vec.at(k);

    estimator.step(measured_rel_pose, joint_angles, contacts);
    k+=freq;
  }






  auto problem = estimator.problem();

  // PrintKeyVector(problem.costs_.keyVector(), "", GTDKeyFormatter);
  // PrintKeyVector(problem.constraintsGraph().keyVector(), "", GTDKeyFormatter);

  std::cout << "init violation: " << problem.constraintsGraph().error(problem.initValues()) << "\n";
  std::cout << "ape: " << estimator.evaluateError(gt_values, problem.initValues(), states.size(), freq) << "\n";

  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  // lm_params.minModelFidelity = 0.3;
  // lm_params.setVerbosityLM("SUMMARY");
  // lm_params.setlambdaUpperBound(1e10);
  // lm_params.setMaxIterations(10);

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result =
      OptimizeSoftConstraints(problem, latex_os, lm_params, 1e4, constraint_unit_scale);
  std::cout << "ape: " << estimator.evaluateError(gt_values, soft_result, states.size(), freq) << "\n";

  // optimize penalty method
  std::cout << "penalty method:\n";
  PenaltyMethodParameters penalty_params;
  penalty_params.lm_parameters = lm_params;
  auto penalty_result =
      OptimizePenaltyMethod(problem, latex_os, penalty_params, constraint_unit_scale);
  std::cout << "ape: " << estimator.evaluateError(gt_values, penalty_result, states.size(), freq) << "\n";

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  AugmentedLagrangianParameters augl_params;
  augl_params.lm_parameters = lm_params;
  auto augl_result =
      OptimizeAugmentedLagrangian(problem, latex_os, augl_params, constraint_unit_scale);
  std::cout << "ape: " << estimator.evaluateError(gt_values, augl_result, states.size(), freq) << "\n";

  // std::cout << "constraint manifold basis variables feasible:\n";
  auto mopt_params = DefaultMoptParamsSV();
  // mopt_params.cc_params->retract_params->setDynamics(true);
  mopt_params.cc_params->retract_params->setProjection(true, 1.0, true);
  // mopt_params.cc_params->retract_params->setFixVars();
  mopt_params.cc_params->retract_params->lm_params.linearSolverType = gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  // mopt_params.cc_params->retract_params->setUopt();
  mopt_params.cc_params->basis_key_func = &FindBasisKeys;
  mopt_params.cc_params->retract_params->check_feasible = true;
  auto cm_result =
        OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)", constraint_unit_scale);
  std::cout << "ape: " << estimator.evaluateError(gt_values, cm_result, states.size(), freq) << "\n";

  std::cout << "constraint manifold basis variables infeasible:\n";
  mopt_params.cc_params->retract_params->lm_params.setMaxIterations(10);
  auto cm_infeas_result =
        OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)", constraint_unit_scale);
  std::cout << "ape: " << estimator.evaluateError(gt_values, cm_infeas_result, states.size(), freq) << "\n";

  std::cout << latex_os.str();


}


int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}

