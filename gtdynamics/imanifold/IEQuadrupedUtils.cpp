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
#include <gtdynamics/manifold/GeneralPriorFactor.h>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
Vector3 get_contact_force(const Pose3 &pose, const Vector6 wrench,
                          OptionalJacobian<3, 6> H_pose,
                          OptionalJacobian<3, 6> H_wrench) {
  Vector3 force_l(wrench(3), wrench(4), wrench(5));
  if (H_pose || H_wrench) {
    gtsam::Matrix36 J_fl_wrench;
    J_fl_wrench << Z_3x3, I_3x3;

    Matrix36 J_rot_pose;
    Rot3 rot = pose.rotation(J_rot_pose);

    Matrix33 H_rot, H_fl;
    Vector3 force_w = rot.rotate(force_l, H_rot, H_fl);

    if (H_pose) {
      *H_pose = H_rot * J_rot_pose;
    }
    if (H_wrench) {
      *H_wrench = H_fl * J_fl_wrench;
    }

    return force_w;
  } else {
    return pose.rotation().rotate(force_l);
  }
}

/* ************************************************************************* */
gtsam::Vector6_ ContactRedundancyConstraint(int t,
                                            const std::vector<int> &contact_ids,
                                            const double &a, const double &b) {
  std::vector<gtsam::Vector6_> error;
  for (size_t i = 0; i < 4; i++) {
    auto link_id = contact_ids.at(i);
    Vector6_ c_wrench(gtdynamics::ContactWrenchKey(link_id, 0, t));
    Pose3_ pose(gtdynamics::PoseKey(link_id, t));
    Vector3_ c_force(get_contact_force, pose, c_wrench);
    gtsam::Matrix63 H;
    if (i == 0) {
      H << 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, a, b, 0;
    } else if (i == 1) {
      H << 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, a, b, 0;
    } else if (i == 2) {
      H << 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, -a, -b, 0;
    } else if (i == 3) {
      H << 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, -a, -b, 0;
    }
    const std::function<gtsam::Vector6(Vector3)> f = [H](const Vector3 &F) {
      return H * F;
    };
    error.emplace_back(gtsam::linearExpression(f, c_force, H));
  }

  return error[0] + error[1] + error[2] + error[3];
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr
ContactRedundancyFactor(int t, const std::vector<int> &contact_ids,
                        const double &a, const double &b,
                        const gtsam::noiseModel::Base::shared_ptr &cost_model,
                        bool express_redundancy) {
  if (express_redundancy) {
    Vector6_ expected_redundancy =
        ContactRedundancyConstraint(t, contact_ids, a, b);
    Vector6_ redundancy(ContactRedundancyKey(t));
    return std::make_shared<ExpressionFactor<Vector6>>(
        cost_model, Vector6::Zero(), expected_redundancy - redundancy);
  } else {
    return std::make_shared<ExpressionFactor<Vector6>>(
        cost_model, Vector6::Zero(),
        ContactRedundancyConstraint(t, contact_ids, a, b));
  }
}

/* ************************************************************************* */
NonlinearFactorGraph
contact_q_factors(const int k, const gtdynamics::PointOnLinks &contact_points,
                  const std::vector<Point3> &contact_in_world,
                  const noiseModel::Base::shared_ptr &cost_model) {
  NonlinearFactorGraph graph;
  // Add contact factors.
  for (size_t contact_idx = 0; contact_idx < contact_points.size();
       contact_idx++) {
    const auto &cp = contact_points.at(contact_idx);
    gtdynamics::FixedContactPointFactor contact_pose_factor(
        gtdynamics::PoseKey(cp.link->id(), k), cost_model,
        contact_in_world.at(contact_idx), cp.point);
    graph.add(contact_pose_factor);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::DynamicsFactors(const size_t k) const {
  NonlinearFactorGraph graph;

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (!link->isFixed()) {
      const auto &connected_joints = link->joints();
      std::vector<gtsam::Key> wrench_keys;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrench_keys.push_back(WrenchKey(i, joint->id(), k));

      // Add wrench keys for contact points.
      for (auto &&cp : contact_points) {
        if (cp.link->id() != i)
          continue;
        auto wrench_key = ContactWrenchKey(i, 0, k);
        wrench_keys.push_back(wrench_key);

        graph.emplace_shared<ContactDynamicsMomentFactor>(
            wrench_key, opt().cm_cost_model,
            gtsam::Pose3(gtsam::Rot3(), -cp.point));
      }

      // add wrench factor for link
      graph.add(WrenchFactor(opt().fa_cost_model, link, wrench_keys, k,
                             params_.gravity));
    }
  }

  for (auto &&joint : robot.joints()) {
    auto j = joint->id(), child_id = joint->child()->id();
    auto const_joint = joint;
    graph.add(WrenchEquivalenceFactor(opt().f_cost_model, const_joint, k));
    graph.add(TorqueFactor(opt().t_cost_model, const_joint, k));
  }
  return graph;
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
NonlinearFactorGraph
IEVision60Robot::getConstraintsGraphStepQ(const int t) const {
  NonlinearFactorGraph graph = graph_builder.qFactors(robot, t);
  graph.add(contact_q_factors(t, contact_points, nominal_contact_in_world,
                              cpoint_cost_model));
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::getConstraintsGraphStepV(const int t) const {
  return graph_builder.vFactors(robot, t, contact_points);
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::getConstraintsGraphStepAD(const int t) const {
  NonlinearFactorGraph graph = graph_builder.aFactors(robot, t, contact_points);
  graph.add(DynamicsFactors(t));
  graph.add(ContactRedundancyFactor(t, contact_ids, nominal_a, nominal_b,
                                    redundancy_model,
                                    params_.express_redundancy));
  return graph;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::frictionConeConstraints(const size_t k) const {
  // NonlinearFactorGraph graph;
  // for (int t = 0; t <= num_steps; t++) {
  //   for (auto &&cp : contact_points) {
  //     int i = cp.link->id();
  //     auto wrench_key = ContactWrenchKey(i, 0, t);

  //     // Add contact dynamics constraints.
  //     graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
  //         PoseKey(i, t), wrench_key, opt().cfriction_cost_model, mu,
  //         gravity);
  //   }
  // }
  // return graph;
  InequalityConstraints constraints;
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::jointLimitConstraints(const size_t k) const {
  InequalityConstraints constraints;
  for (const auto &it : params_.joint_lower_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key joint_key = JointAngleKey(joint_id, k);
    Double_ q_expr(joint_key);
    Double_ q_min_expr = q_expr - Double_(it.second);
    constraints.emplace_shared<DoubleExpressionInequality>(q_min_expr,
                                                           params_.tol_jl);
  }
  for (const auto &it : params_.joint_upper_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key joint_key = JointAngleKey(joint_id, k);
    Double_ q_expr(joint_key);
    Double_ q_max_expr = Double_(it.second) - q_expr;
    constraints.emplace_shared<DoubleExpressionInequality>(q_max_expr,
                                                           params_.tol_jl);
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::torqueLimitConstraints(const size_t k) const {
  InequalityConstraints constraints;
  for (const auto &it : params_.torque_lower_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key torque_key = TorqueKey(joint_id, k);
    Double_ tau_expr(torque_key);
    Double_ tau_min_expr = tau_expr - Double_(it.second);
    constraints.emplace_shared<DoubleExpressionInequality>(tau_min_expr,
                                                           params_.tol_tl);
  }
  for (const auto &it : params_.torque_upper_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key torque_key = TorqueKey(joint_id, k);
    Double_ tau_expr(torque_key);
    Double_ tau_max_expr = Double_(it.second) - tau_expr;
    constraints.emplace_shared<DoubleExpressionInequality>(tau_max_expr,
                                                           params_.tol_tl);
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::collisionAvoidanceConstraints(const size_t k) const {
  InequalityConstraints constraints;
  return constraints;
}

/* ************************************************************************* */
EqualityConstraints IEVision60Robot::eConstraints(const size_t k) const {
  NonlinearFactorGraph graph;
  graph.add(getConstraintsGraphStepQ(k));
  graph.add(getConstraintsGraphStepV(k));
  graph.add(getConstraintsGraphStepAD(k));
  return ConstraintsFromGraph(graph);
}

/* ************************************************************************* */
gtdynamics::EqualityConstraints
IEVision60Robot::initStateConstraints(const Pose3 &init_pose,
                                      const Vector6 &init_twist) const {
  NonlinearFactorGraph graph;
  graph.addPrior<Pose3>(PoseKey(base_id, 0), init_pose, des_pose_nm);
  graph.addPrior<Vector6>(TwistKey(base_id, 0), init_twist, des_twist_nm);
  return ConstraintsFromGraph(graph);
}

/* ************************************************************************* */
InequalityConstraints IEVision60Robot::iConstraints(const size_t k) const {
  InequalityConstraints constraints;
  if (params_.include_friction_cone) {
    constraints.add(frictionConeConstraints(k));
  }
  if (params_.include_joint_limits) {
    constraints.add(jointLimitConstraints(k));
  }
  if (params_.include_torque_limits) {
    constraints.add(torqueLimitConstraints(k));
  }
  if (params_.include_collision_avoidance) {
    constraints.add(collisionAvoidanceConstraints(k));
  }
  return constraints;
}

template <typename CONTAINER>
Values SubValues(const Values &values, const CONTAINER &keys) {
  Values sub_values;
  for (const Key &key : keys) {
    sub_values.insert(key, values.at(key));
  }
  return sub_values;
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
    if (params_.express_redundancy) {
      init_values_t.insert(ContactRedundancyKey(k), zero_vec6);
    }
  }

  Values known_values;
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e20);

  // solve q level
  NonlinearFactorGraph graph_q = getConstraintsGraphStepQ(k);
  graph_q.addPrior<Pose3>(PoseKey(base_id, k), base_pose,
                          graph_builder.opt().p_cost_model);

  Values init_values_q = SubValues(init_values_t, graph_q.keys());
  LevenbergMarquardtOptimizer optimizer_q(graph_q, init_values_q, lm_params);
  auto results_q = optimizer_q.optimize();
  if (graph_q.error(results_q) > 1e-5) {
    std::cout << "solving q fails! error: " << graph_q.error(results_q) << "\n";
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
  }
  known_values.insert(results_v);

  // solve a and dynamics level
  NonlinearFactorGraph graph_ad = getConstraintsGraphStepAD(k);
  graph_ad.addPrior<Vector6>(TwistAccelKey(base_id, k), base_accel,
                             graph_builder.opt().a_cost_model);
  Vector6 zero_vec6 = Vector6::Zero();
  if (params_.express_redundancy) {
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
KeyVector FindBasisKeys4C(const ConnectedComponent::shared_ptr &cc) {
  KeyVector basis_keys;
  for (const Key &key : cc->keys_) {
    auto symb = gtdynamics::DynamicsSymbol(key);
    if (symb.label() == "p" && symb.linkIdx() == 0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "V" && symb.linkIdx() == 0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "A" && symb.linkIdx() == 0) {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}

/* ************************************************************************* */
KeyVector FindBasisKeysReduancy(const ConnectedComponent::shared_ptr &cc) {
  KeyVector basis_keys;
  for (const Key &key : cc->keys_) {
    auto symb = gtdynamics::DynamicsSymbol(key);
    if (symb.label() == "p" && symb.linkIdx() == 0 && symb.time()>0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "V" && symb.linkIdx() == 0 && symb.time()>0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "A" && symb.linkIdx() == 0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "CR") {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}


/* ************************************************************************* */
KeyVector FindBasisKeysTorquesReduancy(const ConnectedComponent::shared_ptr &cc) {
  KeyVector basis_keys;
  for (const Key &key : cc->keys_) {
    auto symb = gtdynamics::DynamicsSymbol(key);
    if (symb.label() == "p" && symb.linkIdx() == 0 && symb.time()>0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "V" && symb.linkIdx() == 0 && symb.time()>0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "T") {
      basis_keys.push_back(key);
    } 
  }
  return basis_keys;
}

/* ************************************************************************* */
BasisKeyFunc IEVision60Robot::getBasisKeyFunc() const {
  if (params_.express_redundancy) {
    if (params_.basis_using_torques) {
      return &FindBasisKeysTorquesReduancy;
    }
    else {
      return &FindBasisKeysReduancy;
    }
  } else {
    return &FindBasisKeys4C;
  }
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
NonlinearFactorGraph IEVision60Robot::collocationCosts(const size_t num_steps,
                                                       double dt) const {
  NonlinearFactorGraph graph;
  for (int t = 0; t < num_steps; t++) {
    graph.add(gtdynamics::FixTimeTrapezoidalPoseCollocationFactor(
        PoseKey(base_id, t), PoseKey(base_id, t + 1), TwistKey(base_id, t),
        TwistKey(base_id, t + 1), dt, graph_builder.opt().pose_col_cost_model));
    graph.add(gtdynamics::FixTimeTrapezoidalTwistCollocationFactor(
        TwistKey(base_id, t), TwistKey(base_id, t + 1),
        TwistAccelKey(base_id, t), TwistAccelKey(base_id, t + 1), dt,
        graph_builder.opt().twist_col_cost_model));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::minTorqueCosts(const size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (int t = 0; t <= num_steps; t++) {
    for (auto &&joint : robot.joints())
      graph.add(gtdynamics::MinTorqueFactor(TorqueKey(joint->id(), t),
                                            min_torque_nm));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::finalStateCosts(const Pose3 &des_pose,
                                 const Vector6 &des_twist,
                                 const size_t num_steps) const {
  NonlinearFactorGraph graph;
  graph.addPrior<Pose3>(PoseKey(base_id, num_steps), des_pose, des_pose_nm);
  graph.addPrior<Vector6>(TwistKey(base_id, num_steps), des_twist,
                          des_twist_nm);
  return graph;
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

/* ************************************************************************* */
template <typename CONTAINER>
void AddLinearPriors(NonlinearFactorGraph &graph, const CONTAINER &keys,
                     const Values &values = Values()) {
  for (const Key &key : keys) {
    gtdynamics::DynamicsSymbol symb(key);
    if (symb.label() == "p") {
      graph.addPrior<Pose3>(
          key, values.exists(key) ? values.at<Pose3>(key) : Pose3(),
          noiseModel::Isotropic::Sigma(6, 1e-2));
    } else if (symb.label() == "q" || symb.label() == "v" ||
               symb.label() == "a" || symb.label() == "T") {
      graph.addPrior<double>(key,
                             values.exists(key) ? values.atDouble(key) : 0.0,
                             noiseModel::Isotropic::Sigma(1, 1e-2));
    } else {
      Vector6 value =
          values.exists(key) ? values.at<Vector6>(key) : Vector6::Zero();
      graph.addPrior<Vector6>(key, value,
                              noiseModel::Isotropic::Sigma(6, 1e-2));
    }
  }
}

/* ************************************************************************* */
template <typename CONTAINER>
void Vision60Retractor::classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                                     KeySet &v_keys, KeySet &ad_keys) {
  for (const Key &key : keys) {
    if (IsQLevel(key)) {
      q_keys.insert(key);
    } else if (IsVLevel(key)) {
      v_keys.insert(key);
    } else {
      ad_keys.insert(key);
    }
  }
}

/* ************************************************************************* */
Vision60Retractor::Vision60Retractor(const IEVision60Robot &robot,
                                     const IEConstraintManifold &manifold,
                                     const Params &params)
    : IERetractor(), robot_(robot), params_(params), graph_q_(), graph_v_(),
      graph_ad_() {

  /// Create merit graph for e-constriants and i-constraints
  merit_graph_ = manifold.eCC()->merit_graph_;
  const InequalityConstraints &i_constraints = *manifold.iConstraints();
  for (const auto &i_constraint : i_constraints) {
    merit_graph_.add(i_constraint->createBarrierFactor(1.0));
  }

  /// Split keys into 3 levels
  KeySet q_keys, v_keys, ad_keys, qv_keys;
  classifyKeys(merit_graph_.keys(), q_keys, v_keys, ad_keys);
  qv_keys = q_keys;
  qv_keys.merge(v_keys);

  /// Split basis keys into 3 levels
  if (params_.use_basis_keys) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.eCC());
    classifyKeys(basis_keys, basis_q_keys_, basis_v_keys_, basis_ad_keys_);
  } else {
    basis_q_keys_ = q_keys;
    basis_v_keys_ = v_keys;
    basis_ad_keys_ = ad_keys;
  }

  /// Split merit graph into 3 levels
  for (const auto &factor : merit_graph_) {
    int lvl = IdentifyLevel(factor->keys());
    if (lvl == 0) {
      graph_q_.add(factor);
    } else if (lvl == 1) {
      graph_v_.add(factor);
    } else {
      graph_ad_.add(factor);
    }
  }

  /// Split i-constraints into 3 levels
  for (size_t i = 0; i < i_constraints.size(); i++) {
    const auto &i_constraint = i_constraints.at(i);
    int lvl = IdentifyLevel(i_constraint->keys());
    if (lvl == 0) {
      i_indices_q_.insert(i);
    } else if (lvl == 1) {
      i_indices_v_.insert(i);
    } else {
      i_indices_ad_.insert(i);
    }
  }

  /// Update factors in v, ad levels as ConstVarFactors
  std::tie(graph_v_, const_var_factors_v_) = ConstVarGraph(graph_v_, q_keys);
  std::tie(graph_ad_, const_var_factors_ad_) =
      ConstVarGraph(graph_ad_, qv_keys);
}

/* ************************************************************************* */
IEConstraintManifold Vision60Retractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {

  Values known_values;
  IndexSet active_indices;
  const Values &values = manifold->values();
  Values new_values = values.retract(delta);
  // Pose3 new_base_pose = Pose(new_values, robot_.base_id, 0);
  // std::cout << "new base pose: \n" << new_base_pose << "\n";
  const InequalityConstraints &i_constraints = *manifold->iConstraints();

  // solve q level with priors
  NonlinearFactorGraph graph_np_q = graph_q_;
  NonlinearFactorGraph graph_wp_q = graph_np_q;
  AddGeneralPriors(new_values, basis_q_keys_, params_.prior_sigma, graph_wp_q);
  Values init_values_q = SubValues(values, graph_wp_q.keys());
  // init_values_q.print("init values:\n", GTDKeyFormatter);
  // for (const auto& factor: graph_wp_q) {
  //   factor->print("", GTDKeyFormatter);
  //   std::cout << "error: " << factor->error(init_values_q) << "\n\n";
  // }
  // graph_wp_q.print("graph q:\n", GTDKeyFormatter);
  LevenbergMarquardtOptimizer optimizer_wp_q(graph_wp_q, init_values_q,
                                             params_.lm_params);
  Values results_q = optimizer_wp_q.optimize();
  // Pose3 results_base_pose = Pose(results_q, robot_.base_id, 0);
  // std::cout << "results base pose: \n" << results_base_pose << "\n";

  // solve q level without priors
  for (const auto &i : i_indices_q_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_q)) {
      active_indices.insert(i);
      graph_np_q.add(i_constraints.at(i)->createL2Factor(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_q(graph_np_q, results_q,
                                             params_.lm_params);
  results_q = optimizer_np_q.optimize();
  known_values.insert(results_q);

  // solve v level with priors
  NonlinearFactorGraph graph_np_v = graph_v_;
  for (auto &factor : const_var_factors_v_) {
    factor->setFixedValues(known_values);
    graph_np_v.add(factor);
  }
  NonlinearFactorGraph graph_wp_v = graph_np_v;
  AddGeneralPriors(new_values, basis_v_keys_, params_.prior_sigma, graph_wp_v);
  Values init_values_v = SubValues(values, graph_wp_v.keys());
  LevenbergMarquardtOptimizer optimizer_wp_v(graph_wp_v, init_values_v,
                                             params_.lm_params);
  Values results_v = optimizer_wp_v.optimize();

  // solve v level without priors
  for (const auto &i : i_indices_v_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_v)) {
      active_indices.insert(i);
      graph_np_v.add(i_constraints.at(i)->createL2Factor(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_v(graph_np_v, results_v,
                                             params_.lm_params);
  results_v = optimizer_np_v.optimize();
  known_values.insert(results_v);

  // solve a and dynamics level with priors
  NonlinearFactorGraph graph_np_ad = graph_ad_;
  for (auto &factor : const_var_factors_ad_) {
    factor->setFixedValues(known_values);
    graph_np_ad.add(factor);
  }
  NonlinearFactorGraph graph_wp_ad = graph_np_ad;
  AddGeneralPriors(new_values, basis_ad_keys_, params_.prior_sigma,
                   graph_wp_ad);
  Values init_values_ad = SubValues(values, graph_wp_ad.keys());
  LevenbergMarquardtOptimizer optimizer_wp_ad(graph_wp_ad, init_values_ad,
                                              params_.lm_params);
  Values results_ad = optimizer_wp_ad.optimize();

  // solve a and dynamics level without priors
  for (const auto &i : i_indices_ad_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_ad)) {
      active_indices.insert(i);
      graph_np_ad.add(i_constraints.at(i)->createL2Factor(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_ad(graph_np_ad, results_ad,
                                              params_.lm_params);
  results_ad = optimizer_np_ad.optimize();
  known_values.insert(results_ad);

  checkFeasible(merit_graph_, known_values);

  return manifold->createWithNewValues(known_values, active_indices);
}

/* ************************************************************************* */
void Vision60Retractor::checkFeasible(const NonlinearFactorGraph &graph,
                                      const Values &values) const {
  if (params_.check_feasible) {
    if (graph.error(values) > params_.feasible_threshold) {
      std::cout << "fail: " << graph.error(values) << "\n";
    }
  }
}

/* ************************************************************************* */
IERetractor::shared_ptr
Vision60RetractorCreator::create(const IEConstraintManifold &manifold) const {
  return std::make_shared<Vision60Retractor>(robot_, manifold, params_);
}

} // namespace gtsam
