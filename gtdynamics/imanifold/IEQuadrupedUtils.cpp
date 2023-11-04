/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/sdf.h>

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtdynamics/utils/DebugUtils.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IEVision60Robot::IEVision60Robot(const Params &_params)
    : params(_params), graph_builder(gtdynamics::DynamicsGraph(
                           getOptSetting(_params), _params.gravity)) {
  des_pose_nm = noiseModel::Isotropic::Sigma(6, params.sigma_des_pose);
  des_twist_nm = noiseModel::Isotropic::Sigma(6, params.sigma_des_twist);
  des_q_nm = noiseModel::Isotropic::Sigma(1, params.sigma_des_twist);
  des_v_nm = noiseModel::Isotropic::Sigma(1, params.sigma_des_twist);
  min_torque_nm = noiseModel::Isotropic::Sigma(1, params.sigma_actuation);
  cpoint_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, params.tol_q);
  redundancy_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params.tol_dynamics);

  // Phase configuration
  for (const auto &idx : params.leaving_indices) {
    leaving_link_indices.insert(legs.at(idx).lower_link_id);
  }
  for (const auto &idx : params.landing_indices) {
    landing_link_indices.insert(legs.at(idx).lower_link_id);
  }
  for (const auto &idx : params.contact_indices) {
    contact_points.emplace_back(
        gtdynamics::PointOnLink(legs.at(idx).lower_link, contact_in_com));
    contact_ids.emplace_back(legs.at(idx).lower_link_id);
  }

  /// Nominal configuration
  int lower0_id = legs[0].lower_link_id;
  int lower1_id = legs[1].lower_link_id;
  int lower2_id = legs[2].lower_link_id;
  int lower3_id = legs[3].lower_link_id;
  nominal_height = gtdynamics::Pose(getNominalConfiguration(), lower0_id, 0)
                       .transformFrom(contact_in_com)
                       .z() *
                   -1;
  nominal_values = getNominalConfiguration(nominal_height);
  nominal_contact_in_world =
      std::vector<Point3>{gtdynamics::Pose(nominal_values, lower0_id, 0)
                              .transformFrom(contact_in_com),
                          gtdynamics::Pose(nominal_values, lower1_id, 0)
                              .transformFrom(contact_in_com),
                          gtdynamics::Pose(nominal_values, lower2_id, 0)
                              .transformFrom(contact_in_com),
                          gtdynamics::Pose(nominal_values, lower3_id, 0)
                              .transformFrom(contact_in_com)};
  nominal_a = 0.5 * (nominal_contact_in_world.at(0).x() -
                     nominal_contact_in_world.at(2).x());
  nominal_b = 0.5 * (nominal_contact_in_world.at(0).y() -
                     nominal_contact_in_world.at(1).y());

  for (const auto &idx : params.contact_indices) {
    contact_in_world.emplace_back(nominal_contact_in_world.at(idx));
  }
}

/* ************************************************************************* */
gtdynamics::Robot IEVision60Robot::getVision60Robot() {
  auto vision60_robot = gtdynamics::CreateRobotFromFile(
      gtdynamics::kUrdfPath + std::string("vision60.urdf"));
  std::vector<std::pair<std::string, std::string>> ordered_link_name_pair{
      {"body", "body"},       {"hip0", "fl_hip"},     {"hip2", "fr_hip"},
      {"hip1", "rl_hip"},     {"hip3", "rr_hip"},     {"upper0", "fl_upper"},
      {"upper2", "fr_upper"}, {"upper1", "rl_upper"}, {"upper3", "rr_upper"},
      {"lower0", "fl_lower"}, {"lower2", "fr_lower"}, {"lower1", "rl_lower"},
      {"lower3", "rr_lower"}};
  std::vector<std::pair<std::string, std::string>> ordered_joint_name_pair{
      {"8", "fl_hip"},   {"10", "fr_hip"},  {"9", "rl_hip"},
      {"11", "rr_hip"},  {"0", "fl_upper"}, {"4", "fr_upper"},
      {"2", "rl_upper"}, {"6", "rr_upper"}, {"1", "fl_lower"},
      {"5", "fr_lower"}, {"3", "rl_lower"}, {"7", "rr_lower"}};
  std::map<std::string, std::string> link_name_map;
  std::map<std::string, std::string> joint_name_map;
  std::vector<std::string> ordered_link_names;
  std::vector<std::string> ordered_joint_names;
  for (const auto &it : ordered_link_name_pair) {
    link_name_map.insert({it.first, it.second});
    ordered_link_names.push_back(it.second);
  }
  for (const auto &it : ordered_joint_name_pair) {
    joint_name_map.insert({it.first, it.second});
    ordered_joint_names.push_back(it.second);
  }
  vision60_robot.renameLinks(link_name_map);
  vision60_robot.renameJoints(joint_name_map);
  vision60_robot.reassignLinks(ordered_link_names);
  vision60_robot.reassignJoints(ordered_joint_names);
  return vision60_robot;
}

/* ************************************************************************* */
std::vector<IEVision60Robot::Leg>
IEVision60Robot::getLegs(const gtdynamics::Robot &robot) {
  std::vector<Leg> legs_;
  std::vector<std::string> leg_names{"fl", "fr", "rl", "rr"};
  for (const auto &leg_name : leg_names) {
    legs_.emplace_back(robot, leg_name + "_hip", leg_name + "_upper",
                       leg_name + "_lower", leg_name + "_hip",
                       leg_name + "_upper", leg_name + "_lower");
  }
  return legs_;
}

/* ************************************************************************* */
gtdynamics::OptimizerSetting
IEVision60Robot::getOptSetting(const Params &params) {
  auto opt = gtdynamics::OptimizerSetting();
  double sigma_dynamics = params.tol_dynamics;
  opt.bp_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.bv_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.ba_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cfriction_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cv_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.ca_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.planar_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.prior_q_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_qv_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_qa_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_t_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.q_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, params.sigma_q_col);
  opt.v_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, params.sigma_v_col);
  opt.time_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.pose_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params.sigma_q_col);
  opt.twist_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params.sigma_v_col);
  return opt;
}

/* ************************************************************************* */
KeyVector
IEVision60Robot::basisKeys(const size_t k,
                           bool include_init_state_constraints) const {
  KeyVector basis_keys;
  // qv levels
  if (k > 0 || !include_init_state_constraints) {
    basis_keys.emplace_back(PoseKey(base_id, k));
    basis_keys.emplace_back(TwistKey(base_id, k));
    for (size_t i = 0; i < 4; i++) {
      if (!params.contact_indices.exists(i)) {
        basis_keys.emplace_back(JointAngleKey(legs[i].hip_joint_id, k));
        basis_keys.emplace_back(JointAngleKey(legs[i].upper_joint_id, k));
        basis_keys.emplace_back(JointAngleKey(legs[i].lower_joint_id, k));
      }
    }
    for (size_t i = 0; i < 4; i++) {
      if (!params.contact_indices.exists(i)) {
        basis_keys.emplace_back(JointVelKey(legs[i].hip_joint_id, k));
        basis_keys.emplace_back(JointVelKey(legs[i].upper_joint_id, k));
        basis_keys.emplace_back(JointVelKey(legs[i].lower_joint_id, k));
      }
    }
  }
  // ad levels
  if (params.leaving_indices.size() == 4) {
     // TODO: for boundary steps
    return basis_keys;
  }
  if (params.ad_basis_using_torques) {
    for (size_t j = 0; j < robot.numJoints(); j++) {
      basis_keys.emplace_back(TorqueKey(j, k));
    }
  } else {
    if (params.contact_indices.size() == 4) {
      // ground
      basis_keys.emplace_back(TwistAccelKey(base_id, k));
      if (params.express_redundancy) {
        basis_keys.emplace_back(ContactRedundancyKey(k));
      }
    }
    else {
      // air
      for (size_t j = 0; j < robot.numJoints(); j++) {
        basis_keys.emplace_back(JointAccelKey(j, k));
      }
    }

  }
  return basis_keys;
}

/* ************************************************************************* */
BasisKeyFunc IEVision60Robot::getBasisKeyFunc() const {
  BasisKeyFunc basis_key_func =
      [=](const ConnectedComponent::shared_ptr &cc) -> KeyVector {
    KeyVector basis_keys;
    size_t k = gtdynamics::DynamicsSymbol(*cc->keys_.begin()).time();
    return basisKeys(k, true);
  };
  return basis_key_func;
}

/* ************************************************************************* */
Values IEVision60Robot::getNominalConfiguration(const double height) const {
  Values qd_values;
  qd_values.insert(PoseKey(base_id, 0),
                   Pose3(Rot3::Identity(), Point3(0, 0, height)));
  for (const auto &joint : robot.joints()) {
    qd_values.insert(JointAngleKey(joint->id(), 0), 0.0);
  }
  std::string base_name = "body";
  Values values = robot.forwardKinematics(qd_values, 0, base_name);

  size_t k = 0;
  Vector6 zero_vec6 = Vector6::Zero();
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    InsertJointAccel(&values, j, k, 0.0);
    InsertTorque(&values, j, k, 0.0);
    InsertWrench(&values, joint->parent()->id(), j, k, zero_vec6);
    InsertWrench(&values, joint->child()->id(), j, k, zero_vec6);
  }
  for (const auto &cp : contact_points) {
    int i = cp.link->id();
    values.insert(ContactWrenchKey(i, 0, k), zero_vec6);
  }
  for (auto &&link : robot.links()) {
    int i = link->id();
    InsertTwistAccel(&values, i, k, zero_vec6);
  }
  if (params.express_redundancy) {
    values.insert(ContactRedundancyKey(k), zero_vec6);
  }
  // PrintKeyVector(values.keys(), "", GTDKeyFormatter);
  return values;
}

/* ************************************************************************* */
void IEVision60Robot::PrintValues(const Values &values,
                                  const size_t num_steps) {
  for (auto &&joint : robot.orderedJoints()) {
    std::cout << joint->name() << "\t";
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto &joint : robot.orderedJoints()) {
      double q = JointAngle(values, joint->id(), k);
      std::cout << q << "\t";
    }
    std::cout << "\n";
  }
  for (auto &&joint : robot.orderedJoints()) {
    std::cout << joint->name() << "\t";
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto &joint : robot.orderedJoints()) {
      double torque = Torque(values, joint->id(), k);
      std::cout << torque << "\t";
    }
    std::cout << "\n";
  }
}

/* ************************************************************************* */
void IEVision60Robot::PrintDelta(const VectorValues &values,
                                 const size_t num_steps) {}

/* ************************************************************************* */
void IEVision60Robot::ExportValues(const Values &values, const size_t num_steps,
                                   const std::string &file_path) {
  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::vector<std::string> jnames;
  for (auto &&joint : robot.joints())
    jnames.push_back(joint->name());
  std::string jnames_str = "";
  for (size_t j = 0; j < jnames.size(); j++) {
    jnames_str += jnames[j] + (j != jnames.size() - 1 ? "," : "");
  }
  std::ofstream traj_file;
  traj_file.open(file_path);
  // angles, vels, accels, torques.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",base_x"
            << ",base_y"
            << ",base_z"
            << ",base_qx"
            << ",base_qy"
            << ",base_qz"
            << ",base_qw"
            << "\n";
  for (int t = 0; t <= num_steps; t++) {
    std::vector<std::string> vals;
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(JointAngle(values, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(JointVel(values, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(JointAccel(values, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(Torque(values, joint->id(), t)));

    Pose3 bp = Pose(values, base_id, t);
    vals.push_back(std::to_string(bp.x()));
    vals.push_back(std::to_string(bp.y()));
    vals.push_back(std::to_string(bp.z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().x()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().y()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().w()));

    std::string vals_str = "";
    for (size_t j = 0; j < vals.size(); j++) {
      vals_str += vals[j] + (j != vals.size() - 1 ? "," : "");
    }
    traj_file << vals_str << "\n";
  }
  traj_file.close();
}

/* ************************************************************************* */
void IEVision60Robot::ExportVector(const VectorValues &values,
                                   const size_t num_steps,
                                   const std::string &file_path) {}

} // namespace gtsam
