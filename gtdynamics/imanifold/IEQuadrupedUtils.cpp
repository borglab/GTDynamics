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
#include <gtdynamics/utils/Initializer.h>
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
  min_torque_nm = noiseModel::Isotropic::Sigma(1, params.sigma_actuation);
  cpoint_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, params.tol_q);
  redundancy_model =
      gtsam::noiseModel::Isotropic::Sigma(6, params.tol_dynamics);

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
Values IEVision60Robot::getInitValuesStep(const size_t k,
                                          const Pose3 &base_pose,
                                          const Vector6 &base_twist,
                                          const Vector6 &base_accel,
                                          Values init_values_t) const {
  if (init_values_t.size() == 0) {
    Vector6 zero_vec6 = Vector6::Zero();
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      InsertJointAngle(&init_values_t, j, k, JointAngle(nominal_values, j));
      InsertJointVel(&init_values_t, j, k, JointVel(nominal_values, j));
      InsertJointAccel(&init_values_t, j, k, 0.0);
      InsertTorque(&init_values_t, j, k, 0.0);
      InsertWrench(&init_values_t, joint->parent()->id(), j, k, zero_vec6);
      InsertWrench(&init_values_t, joint->child()->id(), j, k, zero_vec6);
    }
    for (const auto &cp : contact_points) {
      int i = cp.link->id();
      init_values_t.insert(ContactWrenchKey(i, 0, k), zero_vec6);
    }
    for (auto &&link : robot.links()) {
      int i = link->id();
      InsertPose(&init_values_t, i, k, Pose(nominal_values, i));
      InsertTwist(&init_values_t, i, k, Twist(nominal_values, i));
      InsertTwistAccel(&init_values_t, i, k, zero_vec6);
    }
    if (params.express_redundancy) {
      init_values_t.insert(ContactRedundancyKey(k), zero_vec6);
    }
  }

  Values known_values;
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");

  // solve q level
  NonlinearFactorGraph graph_q = getConstraintsGraphStepQ(k);
  graph_q.addPrior<Pose3>(PoseKey(base_id, k), base_pose,
                          graph_builder.opt().p_cost_model);
  for (size_t i = 0; i < 4; i++) {
    if (!params.contact_indices.exists(i)) {
      Key hip_joint_key = JointAngleKey(legs[i].hip_joint_id, k);
      Key upper_joint_key = JointAngleKey(legs[i].upper_joint_id, k);
      Key lower_joint_key = JointAngleKey(legs[i].lower_joint_id, k);
      graph_q.addPrior<double>(hip_joint_key,
                               init_values_t.atDouble(hip_joint_key),
                               graph_builder.opt().prior_q_cost_model);
      graph_q.addPrior<double>(upper_joint_key,
                               init_values_t.atDouble(upper_joint_key),
                               graph_builder.opt().prior_q_cost_model);
      graph_q.addPrior<double>(lower_joint_key,
                               init_values_t.atDouble(lower_joint_key),
                               graph_builder.opt().prior_q_cost_model);
    }
  }

  Values init_values_q = SubValues(init_values_t, graph_q.keys());
  LevenbergMarquardtOptimizer optimizer_q(graph_q, init_values_q, lm_params);
  auto results_q = optimizer_q.optimize();
  if (graph_q.error(results_q) > 1e-5) {
    std::cout << "solving q fails! error: " << graph_q.error(results_q) << "\n";
    PrintGraphWithError(graph_q, init_values_q);
    PrintGraphWithError(graph_q, results_q);
  }
  known_values.insert(results_q);

  // solve v level
  NonlinearFactorGraph graph_v = getConstraintsGraphStepV(k);
  graph_v.addPrior<Vector6>(TwistKey(base_id, k), base_twist,
                            graph_builder.opt().v_cost_model);
  graph_v = ConstVarGraph(graph_v, known_values);
  Values init_values_v = SubValues(init_values_t, graph_v.keys());
  LevenbergMarquardtOptimizer optimizer_v(graph_v, init_values_v, lm_params);
  auto results_v = optimizer_v.optimize();
  if (graph_v.error(results_v) > 1e-5) {
    std::cout << "solving v fails! error: " << graph_v.error(results_v) << "\n";
    PrintGraphWithError(graph_v, init_values_v);
    PrintGraphWithError(graph_v, results_v);
  }
  known_values.insert(results_v);

  // solve a and dynamics level
  NonlinearFactorGraph graph_ad = getConstraintsGraphStepAD(k);
  // TODO: handle for boundary phases
  if (params.leaving_indices.size()<4) {
    graph_ad.addPrior<Vector6>(TwistAccelKey(base_id, k), base_accel,
                              graph_builder.opt().a_cost_model);
  }

  Vector6 zero_vec6 = Vector6::Zero();
  if (params.express_redundancy) {
    graph_ad.addPrior<Vector6>(ContactRedundancyKey(k), zero_vec6,
                               redundancy_model);
  }
  graph_ad = ConstVarGraph(graph_ad, known_values);
  Values init_values_ad = SubValues(init_values_t, graph_ad.keys());
  LevenbergMarquardtOptimizer optimizer_ad(graph_ad, init_values_ad, lm_params);
  auto results_ad = optimizer_ad.optimize();
  if (graph_ad.error(results_ad) > 1e-5) {
    std::cout << "solving ad fails! error: " << graph_ad.error(results_ad)
              << "\n";
    PrintGraphWithError(graph_ad, init_values_ad);
    PrintGraphWithError(graph_ad, results_ad);
  }
  known_values.insert(results_ad);

  return known_values;
}

/* ************************************************************************* */
Values IEVision60Robot::getInitValuesTrajectory(
    const size_t num_steps, double dt, const Pose3 &base_pose_init,
    const std::vector<gtsam::Pose3> &des_poses,
    std::vector<double> &des_poses_t,
    const std::string initialization_technique) const {
  // Initialize solution.
  gtsam::Values init_vals;
  Initializer initializer;

  // solve 1 step value
  Values init_values_0 = getInitValuesStep(0, base_pose_init);

  // copy for all steps
  if (initialization_technique == "zero") {
    for (int t = 0; t <= num_steps; t++) {
      for (const Key &key : init_values_0.keys()) {
        init_vals.insert(key + t, init_values_0.at(key));
      }
    }
  } else if (initialization_technique == "interp") {
    /// interpolate base pose
    auto interp_values = initializer.InitializeSolutionInterpolationMultiPhase(
        robot, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
        contact_points);
    Values base_link_values;
    for (int t = 0; t <= num_steps; t++) {
      base_link_values.insert(PoseKey(base_id, t),
                              interp_values.at(PoseKey(base_id, t)));
    }
    /// compute base twist
    Vector6 zero_vector6 = Vector6::Zero();
    base_link_values.insert(TwistKey(base_id, 0), zero_vector6);
    for (int t = 0; t < num_steps; t++) {
      Pose3 pose_prev = base_link_values.at<Pose3>(PoseKey(base_id, t));
      Pose3 pose_curr = base_link_values.at<Pose3>(PoseKey(base_id, t + 1));
      Pose3 pose_rel = pose_prev.inverse().compose(pose_curr);
      Vector6 twist_interval = Pose3::Logmap(pose_rel) / dt;
      Vector6 twist_prev = base_link_values.at<Vector6>(TwistKey(base_id, t));
      // Vector6 twist_curr = 2 * twist_interval - twist_prev;
      Vector6 twist_curr = twist_interval;
      base_link_values.insert(TwistKey(base_id, t + 1), twist_curr);
    }
    /// compute base accel
    base_link_values.insert(TwistAccelKey(base_id, 0), zero_vector6);
    for (int t = 0; t < num_steps; t++) {
      Vector6 twist_prev = base_link_values.at<Vector6>(TwistKey(base_id, t));
      Vector6 twist_curr =
          base_link_values.at<Vector6>(TwistKey(base_id, t + 1));
      Vector6 twist_rel = twist_curr - twist_prev;
      Vector6 accel_interval = twist_rel / dt;
      Vector6 accel_prev =
          base_link_values.at<Vector6>(TwistAccelKey(base_id, t));
      // Vector6 accel_curr = 2 * accel_interval - accel_prev;
      Vector6 accel_curr = accel_interval;
      base_link_values.insert(TwistAccelKey(base_id, t + 1), accel_curr);
    }
    /// solve kinodynamics for each step
    Values prev_values = init_values_0;
    init_vals = init_values_0;
    for (int t = 1; t <= num_steps; t++) {
      Values init_values_t;
      for (const Key &key : prev_values.keys()) {
        init_values_t.insert(key + 1, prev_values.at(key));
      }
      Values values_t = getInitValuesStep(
          t, Pose(base_link_values, base_id, t),
          Twist(base_link_values, base_id, t),
          TwistAccel(base_link_values, base_id, t), init_values_t);
      prev_values = values_t;
      init_vals.insert(values_t);

      // std::cout << "pose: " << Pose(init_vals, base_id, t) << "\n";
    }
  }

  return init_vals;
}

/* ************************************************************************* */
BasisKeyFunc IEVision60Robot::getBasisKeyFunc() const {
  BasisKeyFunc basis_key_func =
      [=](const ConnectedComponent::shared_ptr &cc) -> KeyVector {
    KeyVector basis_keys;
    size_t k = gtdynamics::DynamicsSymbol(*cc->keys_.begin()).time();
    if (k > 0) {
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
    if (params.basis_using_torques) {
      // TODO: for boundary steps
      if (params.leaving_indices.size() == 0) {
        for (size_t j = 0; j < robot.numJoints(); j++) {
          basis_keys.emplace_back(TorqueKey(j, k));
        }
      }
    } else {
      basis_keys.emplace_back(TwistAccelKey(base_id, k));
      if (params.express_redundancy) {
        basis_keys.emplace_back(ContactRedundancyKey(k));
      }
    }
    return basis_keys;
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
  Values fk_values = robot.forwardKinematics(qd_values, 0, base_name);
  return fk_values;
}

/* ************************************************************************* */
void IEVision60Robot::PrintValues(const Values &values,
                                  const size_t num_steps) {
  for (auto &&joint : robot.joints()) {
    std::cout << joint->name() << "\t";
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto &joint : robot.joints()) {
      double q = JointAngle(values, joint->id(), k);
      std::cout << q << "\t";
    }
    std::cout << "\n";
  }
  for (auto &&joint : robot.joints()) {
    std::cout << joint->name() << "\t";
  }
  std::cout << std::endl;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto &joint : robot.joints()) {
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
