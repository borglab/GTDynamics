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

#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtdynamics/utils/values.h>

#include <regex>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IEVision60Robot::IEVision60Robot(const Params::shared_ptr &_params,
                                 const PhaseInfo::shared_ptr &_phase_info)
    : params(_params), phase_info(_phase_info),
      graph_builder(gtdynamics::DynamicsGraph(getOptSetting(*_params),
                                              _params->gravity)) {
  des_pose_nm = noiseModel::Isotropic::Sigma(6, params->sigma_des_pose);
  des_twist_nm = noiseModel::Isotropic::Sigma(6, params->sigma_des_twist);
  des_point_nm = noiseModel::Isotropic::Sigma(3, params->sigma_des_point);
  des_point_v_nm = noiseModel::Isotropic::Sigma(3, params->sigma_des_point_v);
  des_q_nm = noiseModel::Isotropic::Sigma(1, params->sigma_des_pose);
  des_v_nm = noiseModel::Isotropic::Sigma(1, params->sigma_des_twist);
  actuation_nm = noiseModel::Isotropic::Sigma(1, params->sigma_actuation);
  jerk_nm = noiseModel::Isotropic::Sigma(1, params->sigma_jerk);
  cf_jerk_nm = noiseModel::Isotropic::Sigma(1, params->sigma_cf_jerk);
  symmetry_nm = noiseModel::Isotropic::Sigma(1, params->sigma_symmetry);
  cpoint_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, params->tol_q);
  c_force_model = gtsam::noiseModel::Isotropic::Sigma(3, params->tol_dynamics);
  redundancy_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params->tol_dynamics);

  // Phase configuration
  for (const auto &idx : phase_info->leaving_indices) {
    leaving_link_indices.insert(legs.at(idx).lower_link_id);
  }
  for (const auto &idx : phase_info->landing_indices) {
    landing_link_indices.insert(legs.at(idx).lower_link_id);
  }
  for (const auto &idx : phase_info->contact_indices) {
    contact_points.emplace_back(
        gtdynamics::PointOnLink(legs.at(idx).lower_link, contact_in_com));
    contact_link_ids.emplace_back(legs.at(idx).lower_link_id);
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

  for (const auto &idx : phase_info->contact_indices) {
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
bool IEVision60Robot::isLeft(const std::string &str) {
  if (str.find("fl_") != std::string::npos) {
    return true;
  }
  if (str.find("rl_") != std::string::npos) {
    return true;
  }
  return false;
}

/* ************************************************************************* */
bool IEVision60Robot::isRight(const std::string& str) {
  if (str.find("fr_") != std::string::npos) {
    return true;
  }
  if (str.find("rr_") != std::string::npos) {
    return true;
  }
  return false;
}

/* ************************************************************************* */
bool IEVision60Robot::isHip(const std::string& str) {
  return str.find("hip") != std::string::npos;
}

/* ************************************************************************* */
std::string IEVision60Robot::counterpart(const std::string &str) {
  if (str.find("fl_") != std::string::npos) {
    return std::regex_replace(str, std::regex("fl_"), "fr_");
  }
  if (str.find("rl_") != std::string::npos) {
    return std::regex_replace(str, std::regex("rl_"), "rr_");
  }
  if (str.find("fr_") != std::string::npos) {
    return std::regex_replace(str, std::regex("fr_"), "fl_");
  }
  if (str.find("rr_") != std::string::npos) {
    return std::regex_replace(str, std::regex("rr_"), "rl_");
  }
  throw std::runtime_error("cannot identify left or right");
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
std::map<std::string, gtsam::Point3> IEVision60Robot::getTorsoCorners() {
  double length = 0.88;
  double width = 0.25;
  double height = 0.19;
  double l = length / 2;
  double w = width / 2;
  double h = height / 2;
  return std::map<std::string, gtsam::Point3>{
      {"fl_top", Point3(l, w, h)},   {"fr_top", Point3(l, -w, h)},
      {"rl_top", Point3(-l, w, h)},  {"rr_top", Point3(-l, -w, h)},
      {"fl_bot", Point3(l, w, -h)},  {"fr_bot", Point3(l, -w, -h)},
      {"rl_bot", Point3(-l, w, -h)}, {"rr_bot", Point3(-l, -w, -h)}};
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
      gtsam::noiseModel::Isotropic::Sigma(6, params.sigma_pose_col);
  opt.twist_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params.sigma_twist_col);
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
      if (!phase_info->contact_indices.exists(i)) {
        basis_keys.emplace_back(JointAngleKey(legs[i].hip_joint_id, k));
        basis_keys.emplace_back(JointAngleKey(legs[i].upper_joint_id, k));
        basis_keys.emplace_back(JointAngleKey(legs[i].lower_joint_id, k));
      }
    }
    for (size_t i = 0; i < 4; i++) {
      if (!phase_info->contact_indices.exists(i)) {
        basis_keys.emplace_back(JointVelKey(legs[i].hip_joint_id, k));
        basis_keys.emplace_back(JointVelKey(legs[i].upper_joint_id, k));
        basis_keys.emplace_back(JointVelKey(legs[i].lower_joint_id, k));
      }
    }
  }
  // ad levels
  if (phase_info->leaving_indices.size() == 4) {
    // TODO: for boundary steps
    return basis_keys;
  }
  if (params->ad_basis_using_torques) {
    for (size_t j = 0; j < robot.numJoints(); j++) {
      basis_keys.emplace_back(TorqueKey(j, k));
    }
  } else {
    if (phase_info->contact_indices.size() == 4) {
      // ground
      basis_keys.emplace_back(TwistAccelKey(base_id, k));
      if (params->express_redundancy) {
        basis_keys.emplace_back(ContactRedundancyKey(k));
      }
    } else {
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
  BasisKeyFunc basis_key_func = [=](const KeyVector &keys) -> KeyVector {
    if (keys.size() == 1) {
      return keys;
    }
    size_t k = gtdynamics::DynamicsSymbol(*keys.begin()).time();
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
  Vector3 zero_vec3 = Vector3::Zero();
  // ad level of joints
  for (auto &&joint : robot.joints()) {
    int j = joint->id();
    InsertJointAccel(&values, j, k, 0.0);
    InsertTorque(&values, j, k, 0.0);
    InsertWrench(&values, joint->parent()->id(), j, k, zero_vec6);
    InsertWrench(&values, joint->child()->id(), j, k, zero_vec6);
  }
  // ad level of links
  for (auto &&link : robot.links()) {
    int i = link->id();
    InsertTwistAccel(&values, i, k, zero_vec6);
  }
  // ad level of contacts
  for (const auto &cp : contact_points) {
    int i = cp.link->id();
    values.insert(ContactWrenchKey(i, 0, k), zero_vec6);
    if (params->express_contact_force) {
      values.insert(ContactForceKey(i, 0, k), zero_vec3);
    }
  }
  if (phase_info->contact_indices.size() == 4 && params->express_redundancy) {
    values.insert(ContactRedundancyKey(k), zero_vec6);
  }
  // PrintKeyVector(values.keys(), "", GTDKeyFormatter);
  return values;
}

void IEVision60Robot::PrintJointValuesLevel(const Values &values,
                                            const size_t num_steps,
                                            const std::string level) {
  std::cout << level << "-level:\n";
  std::cout << "\t";
  for (auto &&joint : robot.orderedJoints()) {
    std::cout << std::setw(10) << joint->name();
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << k << "\t";
    for (const auto &joint : robot.orderedJoints()) {
      double value;
      if (level == "q") {
        value = JointAngle(values, joint->id(), k);
      } else if (level == "v") {
        value = JointVel(values, joint->id(), k);
      } else if (level == "a") {
        value = JointAccel(values, joint->id(), k);
      } else if (level == "d") {
        value = Torque(values, joint->id(), k);
      }
      std::cout << std::setw(10) << std::setprecision(3) << value;
    }
    std::cout << "\n";
  }
}

/* ************************************************************************* */
void IEVision60Robot::PrintTorso(const Values &values, const size_t num_steps) {
  std::cout << "torso:\n";
  std::cout << "\t" << std::setw(54) << "pose" << std::setw(54) << "twist"
            << std::setw(54) << "twist_accel"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << k << "\t";
    Pose3 torso_pose = Pose(values, base_id, k);
    Vector6 torso_twist = Twist(values, base_id, k);
    Vector6 torso_twist_accel = TwistAccel(values, base_id, k);
    for (int i = 0; i < 3; i++) {
      double val = torso_pose.rotation().rpy()(i);
      if (abs(val) < 1e-9) {
        val = 0;
      }
      std::cout << std::setw(9) << std::setprecision(3) << val;
    }
    for (int i = 0; i < 3; i++) {
      double val = torso_pose.translation()(i);
      if (abs(val) < 1e-9) {
        val = 0;
      }
      std::cout << std::setw(9) << std::setprecision(3) << val;
    }
    for (int i = 0; i < 6; i++) {
      double val = torso_twist(i);
      if (abs(val) < 1e-9) {
        val = 0;
      }
      std::cout << std::setw(9) << std::setprecision(3) << val;
    }
    for (int i = 0; i < 6; i++) {
      double val = torso_twist_accel(i);
      if (abs(val) < 1e-9) {
        val = 0;
      }
      std::cout << std::setw(9) << std::setprecision(3) << val;
    }
    std::cout << std::endl;
  }
}

/* ************************************************************************* */
void IEVision60Robot::PrintContactForces(const Values &values,
                                         const size_t num_steps) {
  std::cout << "contact force:\n";
  std::cout << "\t";
  for (const auto &leg : legs) {
    std::cout << std::setw(24) << leg.lower_link->name();
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << k << "\t";
    for (const auto &leg : legs) {
      uint8_t link_id = leg.lower_link->id();
      Key contact_wrench_key = gtdynamics::ContactWrenchKey(link_id, 0, k);
      if (values.exists(contact_wrench_key)) {
        Vector6 wrench_b = values.at<Vector6>(contact_wrench_key);
        Pose3 pose = Pose(values, link_id, k);
        Vector6 wrench_w = pose.inverse().AdjointTranspose(wrench_b);
        for (int i = 3; i < 6; i++) {
          double val = wrench_w(i);
          if (abs(val) < 1e-9) {
            val = 0;
          }
          std::cout << std::setw(8) << std::setprecision(3) << val;
        }
      } else {
        std::cout << std::setw(24) << "-";
      }
    }
    std::cout << std::endl;
  }
}

/* ************************************************************************* */
void IEVision60Robot::PrintValues(const Values &values,
                                  const size_t num_steps) {
  PrintTorso(values, num_steps);
  PrintContactForces(values, num_steps);
  for (const std::string &level : {"q", "v", "a", "d"}) {
    PrintJointValuesLevel(values, num_steps, level);
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
void IEVision60Robot::ExportValuesMultiPhase(
    const Values &values, const std::vector<size_t> &phase_num_steps,
    const std::string &file_path) {
  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::vector<std::string> jnames;
  for (auto &&joint : robot.joints())
    jnames.push_back(joint->name());
  // Joint q,v,a,T,position
  std::string jnames_str_q = "", jnames_str_v = "", jnames_str_a = "",
              jnames_str_T = "", jnames_str_pos = "";
  for (size_t j = 0; j < jnames.size(); j++) {
    const std::string &jname = jnames.at(j);
    jnames_str_q += "," + jname + "_q";
    jnames_str_v += "," + jname + "_v";
    jnames_str_a += "," + jname + "_a";
    jnames_str_T += "," + jname + "_T";
    jnames_str_pos += "," + jname + "_x," + jname + "_y," + jname + "_z";
  }
  // Contact point position and contact force
  std::string contacts_str = "", contacts_force_str = "";
  for (const auto &leg : legs) {
    std::string c_name = leg.lower_link->name() + "_c";
    contacts_str += "," + c_name + "_x," + c_name + "_y," + c_name + "_z";
    contacts_force_str +=
        "," + c_name + "_fx," + c_name + "_fy," + c_name + "_fz";
  }
  // base pose, twist(w), accel(w)
  std::string base_pose_str =
      ",base_x,base_y,base_z,base_qx,base_qy,base_qz,base_qw";
  std::string base_twist_w_str =
      ",base_vroll,base_vpitch,base_vyaw,base_vx,base_vy,base_vz";
  std::string base_accel_w_str =
      ",base_aroll,base_apitch,base_ayaw,base_ax,base_ay,base_az";
  // base corners
  std::string torso_corners_str = "";
  for (const auto &[name, point] : IEVision60Robot::torso_corners_in_com) {
    torso_corners_str += ",torso_" + name + "_x" + ",torso_" + name + "_y" +
                         ",torso_" + name + "_z";
  }

  std::ofstream traj_file;
  traj_file.open(file_path);
  // title
  traj_file << "time" << jnames_str_q << jnames_str_v << jnames_str_a
            << jnames_str_T << jnames_str_pos << contacts_str
            << contacts_force_str << base_pose_str << base_twist_w_str
            << base_accel_w_str << torso_corners_str << "\n";
  size_t num_steps =
      std::accumulate(phase_num_steps.begin(), phase_num_steps.end(), 0);
  std::vector<double> step_time{0};
  double time = 0;
  for (size_t i = 0; i < phase_num_steps.size(); i++) {
    for (size_t k = 0; k < phase_num_steps[i]; k++) {
      time += values.atDouble(PhaseKey(i));
      step_time.push_back(time);
    }
  }
  // Joint q,v,a,T,position
  for (int t = 0; t <= num_steps; t++) {
    std::vector<double> vals;
    vals.push_back(step_time[t]);
    for (auto &&joint : robot.joints())
      vals.push_back(JointAngle(values, joint->id(), t));
    for (auto &&joint : robot.joints())
      vals.push_back(JointVel(values, joint->id(), t));
    for (auto &&joint : robot.joints())
      vals.push_back(JointAccel(values, joint->id(), t));
    for (auto &&joint : robot.joints())
      vals.push_back(Torque(values, joint->id(), t));
    for (auto &&joint : robot.joints()) {
      Pose3 child_pose = Pose(values, joint->child()->id(), t);
      Pose3 joint_pose = child_pose * joint->jMc().inverse();
      vals.insert(vals.end(), {joint_pose.x(), joint_pose.y(), joint_pose.z()});
    }

    // Contact point position and contact force
    for (const auto &leg : legs) {
      Pose3 lower_pose = Pose(values, leg.lower_link_id, t);
      Point3 cp_w = lower_pose.transformFrom(contact_in_com);
      vals.insert(vals.end(), {cp_w.x(), cp_w.y(), cp_w.z()});
    }
    for (const auto &leg : legs) {
      Key contact_force_key = ContactForceKey(leg.lower_link_id, 0, t);
      Vector3 f = Vector3::Zero();
      if (values.exists(contact_force_key)) {
        f = values.at<Vector3>(contact_force_key);
      }
      vals.insert(vals.end(), {f.x(), f.y(), f.z()});
    }
    // base pose, twist(w), accel(w)
    Pose3 pose = Pose(values, base_id, t);
    Rot3 rot = pose.rotation();
    auto quat = rot.toQuaternion();
    vals.insert(vals.end(), {pose.x(), pose.y(), pose.z()});
    vals.insert(vals.end(), {quat.x(), quat.y(), quat.z(), quat.w()});
    Vector6 twist = Twist(values, base_id, t);
    Vector6 twist_accel = TwistAccel(values, base_id, t);
    Pose3 rot_pose(rot, Point3::Zero());
    Vector6 twist_w = rot_pose.AdjointMap() * twist;
    Vector6 twist_accel_w = rot_pose.AdjointMap() * twist_accel;
    vals.insert(vals.end(), {twist_w(0), twist_w(1), twist_w(2), twist_w(3),
                             twist_w(4), twist_w(5)});
    vals.insert(vals.end(),
                {twist_accel_w(0), twist_accel_w(1), twist_accel_w(2),
                 twist_accel_w(3), twist_accel_w(4), twist_accel_w(5)});
    for (const auto &[name, point] : IEVision60Robot::torso_corners_in_com) {
      Point3 point_w = pose.transformFrom(point);
      vals.insert(vals.end(), {point_w.x(), point_w.y(), point_w.z()});
    }

    std::string vals_str = "";
    for (size_t j = 0; j < vals.size(); j++) {
      vals_str += std::to_string(vals[j]) + (j != vals.size() - 1 ? "," : "");
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
