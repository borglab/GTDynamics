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

#include "utils/DynamicsSymbol.h"
#include <gtdynamics/utils/QuadrupedUtils.h>
#include <gtsam/base/Vector.h>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/algorithm/string/join.hpp>

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
    return boost::make_shared<ExpressionFactor<Vector6>>(
        cost_model, Vector6::Zero(), expected_redundancy - redundancy);
  } else {
    return boost::make_shared<ExpressionFactor<Vector6>>(
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
NonlinearFactorGraph Vision60Robot::DynamicsFactors(
    const int k, const boost::optional<PointOnLinks> &contact_points) const {
  NonlinearFactorGraph graph;

  for (auto &&link : robot.links()) {
    int i = link->id();
    if (!link->isFixed()) {
      const auto &connected_joints = link->joints();
      std::vector<DynamicsSymbol> wrench_keys;

      // Add wrench keys for joints.
      for (auto &&joint : connected_joints)
        wrench_keys.push_back(WrenchKey(i, joint->id(), k));

      // Add wrench keys for contact points.
      if (contact_points) {
        for (auto &&cp : *contact_points) {
          if (cp.link->id() != i)
            continue;
          // TODO(frank): allow multiple contact points on one link, id = 0,1,..
          auto wrench_key = ContactWrenchKey(i, 0, k);
          wrench_keys.push_back(wrench_key);

          graph.emplace_shared<ContactDynamicsMomentFactor>(
              wrench_key, opt().cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -cp.point));
        }
      }

      // add wrench factor for link
      graph.add(
          WrenchFactor(opt().fa_cost_model, link, wrench_keys, k, gravity));
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
    if (symb.label() == "p" && symb.linkIdx() == 0) {
      basis_keys.push_back(key);
    } else if (symb.label() == "V" && symb.linkIdx() == 0) {
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
gtdynamics::OptimizerSetting Vision60Robot::getOptSetting() const {
  auto opt = gtdynamics::OptimizerSetting();
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
  opt.q_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.v_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.time_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.pose_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.twist_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  return opt;
}

/* ************************************************************************* */
NonlinearFactorGraph
Vision60Robot::getConstraintsGraphStepQ(const int t) const {
  NonlinearFactorGraph graph = graph_builder.qFactors(robot, t);
  graph.add(contact_q_factors(t, contact_points, contact_in_world,
                              cpoint_cost_model));
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
Vision60Robot::getConstraintsGraphStepV(const int t) const {
  return graph_builder.vFactors(robot, t, contact_points);
}

/* ************************************************************************* */
NonlinearFactorGraph
Vision60Robot::getConstraintsGraphStepAD(const int t) const {
  NonlinearFactorGraph graph = graph_builder.aFactors(robot, t, contact_points);
  graph.add(DynamicsFactors(t, contact_points));
  graph.add(ContactRedundancyFactor(t, contact_ids, a, b, redundancy_model,
                                    express_redundancy));
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph Vision60Robot::getConstraintsGraphStep(const int t) const {
  NonlinearFactorGraph graph;
  graph.add(getConstraintsGraphStepQ(t));
  graph.add(getConstraintsGraphStepV(t));
  graph.add(getConstraintsGraphStepAD(t));
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
Vision60Robot::getConstraintsGraphTrajectory(const int num_steps) const {
  NonlinearFactorGraph graph;
  for (int t = 0; t <= num_steps; t++) {
    graph.add(getConstraintsGraphStep(t));
  }
  return graph;
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
Values Vision60Robot::getInitValuesStep(const int t, const Pose3 &base_pose,
                                        const Vector6 &base_twist,
                                        const Vector6 &base_accel,
                                        Values init_values_t) const {
  if (init_values_t.size() == 0) {
    init_values_t = nominal_values;
    Vector6 zero_vec6 = Vector6::Zero();
    for (auto &&joint : robot.joints()) {
      int j = joint->id();
      InsertJointAccel(&init_values_t, j, 0.0);
      InsertTorque(&init_values_t, j, 0.0);
      InsertWrench(&init_values_t, joint->parent()->id(), j, zero_vec6);
      InsertWrench(&init_values_t, joint->child()->id(), j, zero_vec6);
    }
    for (const auto &cp : contact_points) {
      int i = cp.link->id();
      init_values_t.insert(ContactWrenchKey(i, 0, 0), zero_vec6);
    }
    for (auto &&link : robot.links()) {
      InsertTwistAccel(&init_values_t, link->id(), zero_vec6);
    }
    if (express_redundancy) {
      init_values_t.insert(ContactRedundancyKey(t), zero_vec6);
    }
  }

  Values known_values;
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e20);

  // solve q level
  NonlinearFactorGraph graph_q = getConstraintsGraphStepQ(t);
  graph_q.addPrior<Pose3>(PoseKey(base_id, t), base_pose,
                          graph_builder.opt().p_cost_model);

  Values init_values_q = SubValues(init_values_t, graph_q.keys());
  LevenbergMarquardtOptimizer optimizer_q(graph_q, init_values_q, lm_params);
  auto results_q = optimizer_q.optimize();
  if (graph_q.error(results_q) > 1e-5) {
    std::cout << "solving q fails! error: " << graph_q.error(results_q) << "\n";
  }
  known_values.insert(results_q);

  // solve v level
  NonlinearFactorGraph graph_v = getConstraintsGraphStepV(t);
  graph_v.addPrior<Vector6>(TwistKey(base_id, t), base_twist,
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
  NonlinearFactorGraph graph_ad = getConstraintsGraphStepAD(t);
  graph_ad.addPrior<Vector6>(TwistAccelKey(base_id, t), base_accel,
                             graph_builder.opt().a_cost_model);
  Vector6 zero_vec6 = Vector6::Zero();
  if (express_redundancy) {
    graph_ad.addPrior<Vector6>(ContactRedundancyKey(t), zero_vec6,
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
Values Vision60Robot::getInitValuesTrajectory(
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

  // if (initialization_technique == "interp")
  //   // TODO(aescontrela): Figure out why the linearly interpolated initial
  //   // trajectory fails to optimize. My initial guess is that the optimizer
  //   has
  //   // a difficult time optimizing the trajectory when the initial solution
  //   lies
  //   // in the infeasible region. This would make sense if I were using an IPM
  //   to
  //   // solve this problem...
  //   init_vals = initializer.InitializeSolutionInterpolationMultiPhase(
  //       robot, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
  //       contact_points);
  // else if (initialization_technique == "zeros")
  //   init_vals = initializer.ZeroValuesTrajectory(robot, t_steps, 0, 0.0,
  //   contact_points);
  // else if (initialization_technique == "inverse_kinematics")
  //   init_vals = initializer.InitializeSolutionInverseKinematics(
  //       robot, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
  //       contact_points);

  return init_vals;
}

/* ************************************************************************* */
Values Vision60Robot::getNominalConfiguration(const double height) const {
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
NonlinearFactorGraph Vision60Robot::collocationCosts(const int num_steps,
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
NonlinearFactorGraph Vision60Robot::minTorqueCosts(const int num_steps) const {
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
Vision60Robot::frictionConeCosts(const int num_steps) const {
  NonlinearFactorGraph graph;
  for (int t = 0; t <= num_steps; t++) {
    for (auto &&cp : contact_points) {
      int i = cp.link->id();
      auto wrench_key = ContactWrenchKey(i, 0, t);

      // Add contact dynamics constraints.
      graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
          PoseKey(i, t), wrench_key, opt().cfriction_cost_model, mu, gravity);
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
Vision60Robot::boundaryCosts(const Pose3 &init_pose, const Vector6 &init_twist,
                             const std::vector<Pose3> &des_poses,
                             const std::vector<double> &des_poses_t,
                             double dt) const {
  NonlinearFactorGraph graph;

  graph.addPrior<Pose3>(PoseKey(base_id, 0), init_pose,
                        gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics));
  graph.addPrior<Vector6>(
      TwistKey(base_id, 0), init_twist,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics));
  for (size_t i = 0; i < des_poses.size(); i++)
    graph.addPrior(
        PoseKey(base_id, static_cast<int>(std::ceil(des_poses_t[i] / dt))),
        des_poses[i], des_pose_nm);

  return graph;
}

/* ************************************************************************* */
void Vision60Robot::printJointAngles(const Values &values, int t) const {
  for (const auto &joint : robot.joints()) {
    double q = JointAngle(values, joint->id(), t);
    std::cout << joint->name() << "\t" << joint->parent()->name() << "\t"
              << joint->child()->name() << "\t" << q << "\n";
  }
}

/* ************************************************************************* */
void Vision60Robot::exportTrajectory(const Values &results,
                                     const size_t num_steps,
                                     std::string file_path) {
  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::vector<std::string> jnames;
  for (auto &&joint : robot.joints())
    jnames.push_back(joint->name());
  std::string jnames_str = boost::algorithm::join(jnames, ",");
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
      vals.push_back(std::to_string(JointAngle(results, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(JointVel(results, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(JointAccel(results, joint->id(), t)));
    for (auto &&joint : robot.joints())
      vals.push_back(std::to_string(Torque(results, joint->id(), t)));

    Pose3 bp = Pose(results, base_id, t);
    vals.push_back(std::to_string(bp.x()));
    vals.push_back(std::to_string(bp.y()));
    vals.push_back(std::to_string(bp.z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().x()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().y()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().w()));

    std::string vals_str = boost::algorithm::join(vals, ",");
    traj_file << vals_str << "\n";
  }
  traj_file.close();
}

} // namespace gtsam
