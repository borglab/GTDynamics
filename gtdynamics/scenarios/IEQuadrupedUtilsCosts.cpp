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

#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactPointFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

using namespace gtdynamics;

namespace gtsam {
/* ************************************************************************* */
Vector3 IEVision60Robot::GetContactForce(const Pose3 &pose,
                                         const Vector6 wrench,
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
NoiseModelFactor::shared_ptr
IEVision60Robot::contactForceFactor(const uint8_t link_id,
                                    const size_t k) const {
  Vector6_ c_wrench(gtdynamics::ContactWrenchKey(link_id, 0, k));
  Pose3_ pose(gtdynamics::PoseKey(link_id, k));
  Vector3_ expected_contact_force_expr(IEVision60Robot::GetContactForce, pose,
                                       c_wrench);
  Vector3_ contact_force_expr(ContactForceKey(link_id, 0, k));
  return std::make_shared<ExpressionFactor<Vector3>>(
      c_force_model, Vector3::Zero(),
      expected_contact_force_expr - contact_force_expr);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr
IEVision60Robot::contactRedundancyFactor(const size_t k) const {
  const double &a = nominal_a;
  const double &b = nominal_b;
  std::vector<gtsam::Vector6_> error;
  for (size_t i = 0; i < 4; i++) {
    gtsam::Matrix63 H;
    if (i == 0) {
      H << 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, a, b, 0;
    } else if (i == 1) {
      H << 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, -a, -b, 0;
    } else if (i == 2) {
      H << 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, a, b, 0;
    } else if (i == 3) {
      H << 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, -a, -b, 0;
    }
    const std::function<gtsam::Vector6(Vector3)> f = [H](const Vector3 &F) {
      return H * F;
    };
    auto link_id = contact_link_ids.at(i);
    if (params->express_contact_force) {
      Vector3_ c_force(ContactForceKey(link_id, 0, k));
      error.emplace_back(gtsam::linearExpression(f, c_force, H));
    } else {
      Vector6_ c_wrench(gtdynamics::ContactWrenchKey(link_id, 0, k));
      Pose3_ pose(gtdynamics::PoseKey(link_id, k));
      Vector3_ c_force(IEVision60Robot::GetContactForce, pose, c_wrench);
      error.emplace_back(gtsam::linearExpression(f, c_force, H));
    }
  }
  Vector6_ redundancy_expr = error[0] + error[1] + error[2] + error[3];

  if (params->express_redundancy) {
    Vector6_ redundancy(ContactRedundancyKey(k));
    return std::make_shared<ExpressionFactor<Vector6>>(
        redundancy_model, Vector6::Zero(), redundancy_expr - redundancy);
  } else {
    return std::make_shared<ExpressionFactor<Vector6>>(
        redundancy_model, Vector6::Zero(), redundancy_expr);
  }
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::qPointContactFactors(const size_t k) const {
  NonlinearFactorGraph graph;
  // Add contact factors.
  for (size_t contact_idx = 0; contact_idx < contact_points.size();
       contact_idx++) {
    const auto &cp = contact_points.at(contact_idx);
    gtdynamics::FixedContactPointFactor contact_pose_factor(
        gtdynamics::PoseKey(cp.link->id(), k), cpoint_cost_model,
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

        if (params->express_contact_force) {
          graph.add(contactForceFactor(i, k));
        }

        if (leaving_link_indices.exists(i)) {
          graph.addPrior<Vector6>(wrench_key, Vector6::Zero(),
                                  opt().f_cost_model);
        } else {
          graph.emplace_shared<ContactDynamicsMomentFactor>(
              wrench_key, opt().cm_cost_model,
              gtsam::Pose3(gtsam::Rot3(), -cp.point));
        }
      }

      // add wrench factor for link
      graph.add(WrenchFactor(opt().fa_cost_model, link, wrench_keys, k,
                             params->gravity));
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
NonlinearFactorGraph
IEVision60Robot::getConstraintsGraphStepQ(const int t) const {
  NonlinearFactorGraph graph = graph_builder.qFactors(robot, t);
  graph.add(qPointContactFactors(t));
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

  NonlinearFactorGraph graph;
  if (params->boundary_constrain_a) {
    graph = graph_builder.aFactors(robot, t, contact_points);
  } else {
    PointOnLinks static_contact_points;
    for (const auto &point_on_link : contact_points) {
      if (!leaving_link_indices.exists(point_on_link.link->id())) {
        static_contact_points.emplace_back(point_on_link);
      }
    }
    graph = graph_builder.aFactors(robot, t, static_contact_points);
  }
  graph.add(DynamicsFactors(t));
  if (contact_link_ids.size() == 4) {
    graph.add(contactRedundancyFactor(t));
  }

  return graph;
}

/* ************************************************************************* */
gtdynamics::DoubleExpressionInequality::shared_ptr
IEVision60Robot::frictionConeConstraint(const size_t contact_link_id,
                                        const size_t k) const {
  double mu = params->mu;
  double mu_prime = mu * mu;
  auto friction_cone_function =
      [mu_prime](const Vector3 &f, gtsam::OptionalJacobian<1, 3> H = {}) {
        const double &fx = f(0), fy = f(1), fz = f(2);
        int sign_fz = (fz > 0) - (fz < 0);
        double result = sign_fz * mu_prime * fz * fz - fx * fx - fy * fy;
        if (H) {
          (*H) << -2 * fx, -2 * fy, 2 * sign_fz * mu_prime * fz;
        }
        return result;
      };

  Vector3_ contact_force_expr(ContactForceKey(contact_link_id, 0, k));
  Double_ compute_fc_expr(friction_cone_function, contact_force_expr);
  return std::make_shared<DoubleExpressionInequality>(compute_fc_expr,
                                                      params->tol_fc);
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::frictionConeConstraints(const size_t k) const {
  InequalityConstraints constraints;
  if (params->i_constraints_symmetry) {
    for (const auto &link_id : contact_link_ids) {
      if (leaving_link_indices.exists(link_id)) {
        continue;
      }
      std::string link_name = robot.orderedLinks().at(link_id)->name();
      if (isLeft(link_name)) {
        std::string counterpart_name = counterpart(link_name);
        size_t counterpart_id = robot.link(counterpart_name)->id();
        auto constraint = frictionConeConstraint(link_id, k);
        auto other_constraint = frictionConeConstraint(counterpart_id, k);
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, other_constraint);
      }
    }
  } else {
    for (const auto &link_id : contact_link_ids) {
      if (leaving_link_indices.exists(link_id)) {
        continue;
      }
      constraints.push_back(frictionConeConstraint(link_id, k));
    }
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::DoubleExpressionInequality::shared_ptr
IEVision60Robot::obstacleCollisionFreeConstraint(const size_t link_idx,
                                                 const size_t k,
                                                 const Point3 &p_l,
                                                 const Point3 &center,
                                                 const double radius) const {
  Pose3_ wTl(PoseKey(link_idx, k));
  Point3_ p_l_const(p_l);
  Point3_ p_w(wTl, &Pose3::transformFrom, p_l_const);
  Point3_ center_const(center);
  Double_ dist(distance3, p_w, center_const);
  Double_ collsion_free_expr = dist - Double_(radius);
  return std::make_shared<DoubleExpressionInequality>(collsion_free_expr,
                                                      params->tol_cf);
}

/* ************************************************************************* */
gtdynamics::DoubleExpressionInequality::shared_ptr
IEVision60Robot::groundCollisionFreeConstraint(const size_t link_idx,
                                               const size_t k,
                                               const Point3 &p_l) const {
  Pose3_ wTl(PoseKey(link_idx, k));
  Point3_ p_l_const(p_l);
  Point3_ p_w(wTl, &Pose3::transformFrom, p_l_const);
  Double_ z(&point3_z, p_w);
  return std::make_shared<DoubleExpressionInequality>(z, params->tol_cf);
}

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
IEVision60Robot::jointUpperLimitConstraint(const std::string &j_name,
                                           const size_t k,
                                           const double upper_limit) const {
  size_t joint_id = robot.joint(j_name)->id();
  Key joint_key = JointAngleKey(joint_id, k);
  Double_ q_expr(joint_key);
  Double_ q_max_expr = Double_(upper_limit) - q_expr;
  return std::make_shared<DoubleExpressionInequality>(q_max_expr,
                                                      params->tol_jl);
}

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
IEVision60Robot::jointLowerLimitConstraint(const std::string &j_name,
                                           const size_t k,
                                           const double lower_limit) const {
  size_t joint_id = robot.joint(j_name)->id();
  Key joint_key = JointAngleKey(joint_id, k);
  Double_ q_expr(joint_key);
  Double_ q_min_expr = q_expr - Double_(lower_limit);
  return std::make_shared<DoubleExpressionInequality>(q_min_expr,
                                                      params->tol_jl);
}

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
IEVision60Robot::torqueUpperLimitConstraint(const std::string &j_name,
                                            const size_t k,
                                            const double upper_limit) const {
  size_t joint_id = robot.joint(j_name)->id();
  Key torque_key = TorqueKey(joint_id, k);
  Double_ tau_expr(torque_key);
  Double_ tau_max_expr = Double_(upper_limit) - tau_expr;
  return std::make_shared<DoubleExpressionInequality>(tau_max_expr,
                                                      params->tol_tl);
}

/* ************************************************************************* */
DoubleExpressionInequality::shared_ptr
IEVision60Robot::torqueLowerLimitConstraint(const std::string &j_name,
                                            const size_t k,
                                            const double lower_limit) const {
  size_t joint_id = robot.joint(j_name)->id();
  Key torque_key = TorqueKey(joint_id, k);
  Double_ tau_expr(torque_key);
  Double_ tau_min_expr = tau_expr - Double_(lower_limit);
  return std::make_shared<DoubleExpressionInequality>(tau_min_expr,
                                                      params->tol_tl);
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::jointLimitConstraints(const size_t k) const {
  InequalityConstraints constraints;

  if (params->i_constraints_symmetry) {
    for (const auto &[j_name, lower_limit] : params->joint_lower_limits) {
      if (isLeft(j_name)) {
        auto constraint = jointLowerLimitConstraint(j_name, k, lower_limit);
        std::string counterpart_name = counterpart(j_name);
        DoubleExpressionInequality::shared_ptr other_constraint;
        if (isHip(j_name)) {
          other_constraint = jointUpperLimitConstraint(
              counterpart_name, k,
              params->joint_upper_limits.at(counterpart_name));
        } else {
          other_constraint = jointLowerLimitConstraint(
              counterpart_name, k,
              params->joint_lower_limits.at(counterpart_name));
        }
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, other_constraint);
      }
    }
    for (const auto &[j_name, upper_limit] : params->joint_upper_limits) {
      if (isLeft(j_name)) {
        auto constraint = jointUpperLimitConstraint(j_name, k, upper_limit);
        std::string counterpart_name = counterpart(j_name);
        DoubleExpressionInequality::shared_ptr other_constraint;
        if (isHip(j_name)) {
          other_constraint = jointLowerLimitConstraint(
              counterpart_name, k,
              params->joint_lower_limits.at(counterpart_name));
        } else {
          other_constraint = jointUpperLimitConstraint(
              counterpart_name, k,
              params->joint_upper_limits.at(counterpart_name));
        }
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, other_constraint);
      }
    }
  }

  else {
    for (const auto &[j_name, lower_limit] : params->joint_lower_limits) {
      constraints.push_back(jointLowerLimitConstraint(j_name, k, lower_limit));
    }
    for (const auto &[j_name, upper_limit] : params->joint_upper_limits) {
      constraints.push_back(jointUpperLimitConstraint(j_name, k, upper_limit));
    }
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::torqueLimitConstraints(const size_t k) const {
  InequalityConstraints constraints;

  if (params->i_constraints_symmetry) {
    for (const auto &[j_name, lower_limit] : params->torque_lower_limits) {
      if (isLeft(j_name)) {
        auto constraint = torqueLowerLimitConstraint(j_name, k, lower_limit);
        std::string counterpart_name = counterpart(j_name);
        DoubleExpressionInequality::shared_ptr other_constraint;
        if (isHip(j_name)) {
          other_constraint = torqueUpperLimitConstraint(
              counterpart_name, k,
              params->torque_upper_limits.at(counterpart_name));
        } else {
          other_constraint = torqueLowerLimitConstraint(
              counterpart_name, k,
              params->torque_lower_limits.at(counterpart_name));
        }
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, other_constraint);
      }
    }
    for (const auto &[j_name, upper_limit] : params->torque_upper_limits) {
      if (isLeft(j_name)) {
        auto constraint = torqueUpperLimitConstraint(j_name, k, upper_limit);
        std::string counterpart_name = counterpart(j_name);
        DoubleExpressionInequality::shared_ptr other_constraint;
        if (isHip(j_name)) {
          other_constraint = torqueLowerLimitConstraint(
              counterpart_name, k,
              params->torque_lower_limits.at(counterpart_name));
        } else {
          other_constraint = torqueUpperLimitConstraint(
              counterpart_name, k,
              params->torque_upper_limits.at(counterpart_name));
        }
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, other_constraint);
      }
    }
  } else {
    for (const auto &[j_name, lower_limit] : params->torque_lower_limits) {
      constraints.push_back(torqueLowerLimitConstraint(j_name, k, lower_limit));
    }
    for (const auto &[j_name, upper_limit] : params->torque_upper_limits) {
      constraints.push_back(torqueUpperLimitConstraint(j_name, k, upper_limit));
    }
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::obstacleCollisionFreeConstraints(const size_t k) const {
  InequalityConstraints constraints;

  for (const auto &[center, radius] : params->sphere_obstacles) {
    for (const auto &[link_name, p_l] : params->collision_checking_points_s) {
      uint8_t link_idx = robot.link(link_name)->id();
      constraints.push_back(
          obstacleCollisionFreeConstraint(link_idx, k, p_l, center, radius));
    }
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::groundCollisionFreeConstraints(const size_t k) const {
  InequalityConstraints constraints;
  if (params->i_constraints_symmetry) {
    for (const auto &[link_name, p_l] : params->collision_checking_points_z) {
      if (isRight(link_name)) {
        continue;
      }
      uint8_t link_idx = robot.link(link_name)->id();
      auto constraint = groundCollisionFreeConstraint(link_idx, k, p_l);
      if (isLeft(link_name)) {
        std::string counterpart_name = counterpart(link_name);
        size_t counterpart_idx = robot.link(counterpart_name)->id();
        auto counterpart_constraint =
            groundCollisionFreeConstraint(counterpart_idx, k, p_l);
        constraints.emplace_shared<TwinDoubleExpressionInequality>(
            constraint, counterpart_constraint);
      } else {
        constraints.push_back(constraint);
      }
    }
  } else {
    for (const auto &[link_name, p_l] : params->collision_checking_points_z) {
      uint8_t link_idx = robot.link(link_name)->id();
      constraints.push_back(groundCollisionFreeConstraint(link_idx, k, p_l));
    }
  }
  return constraints;
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr IEVision60Robot::statePointCostFactor(
    const size_t link_id, const Point3 &point_l, const Point3 &point_w,
    const size_t k) const {
  Pose3_ pose_expr(PoseKey(link_id, k));
  Point3_ point_l_expr(point_l);
  Point3_ point_w_expr(pose_expr, &Pose3::transformFrom, point_l_expr);
  return std::make_shared<ExpressionFactor<Point3>>(des_point_nm, point_w,
                                                    point_w_expr);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr IEVision60Robot::statePointVelCostFactor(
    const size_t link_id, const Point3 &point_l, const Vector3 &vel_w,
    const size_t k) const {
  Pose3_ pose_expr(PoseKey(link_id, k));
  Vector6_ twist_expr(TwistKey(link_id, k));

  Pose3 lTc(Rot3::Identity(), point_l);
  Pose3 cTl = lTc.inverse();
  Matrix36 H_vel_c;
  H_vel_c << Z_3x3, I_3x3;
  H_vel_c = H_vel_c * cTl.AdjointMap();
  const std::function<gtsam::Vector3(gtsam::Vector6)> f =
      [H_vel_c](const gtsam::Vector6 &A) { return H_vel_c * A; };
  Vector3_ vel_l_expr = gtsam::linearExpression(f, twist_expr, H_vel_c);
  Rot3_ rot_expr(&Pose3::rotation, pose_expr);
  Vector3_ vel_w_expr(rot_expr, &Rot3::rotate, vel_l_expr);
  return std::make_shared<ExpressionFactor<Vector3>>(des_point_v_nm, vel_w,
                                                     vel_w_expr);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr
IEVision60Robot::contactJerkCostFactor(const size_t link_id,
                                       const size_t k) const {
  Vector3_ cf_curr(ContactForceKey(link_id, 0, k));
  Vector3_ cf_next(ContactForceKey(link_id, 0, k + 1));
  Vector3_ cf_diff = cf_next - cf_curr;
  Double_ cf_diff_norm(&norm3, cf_diff);
  auto ramp_func = RampFunction(params->cf_jerk_threshold);
  Double_ error(ramp_func, cf_diff_norm);
  return std::make_shared<ExpressionFactor<double>>(cf_jerk_nm, 0.0, error);
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::contactJerkCostFactors(const size_t k) const {
  NonlinearFactorGraph graph;
  for (const auto &link_id : contact_link_ids) {
    graph.add(contactJerkCostFactor(link_id, k));
  }
  return graph;
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
gtdynamics::EqualityConstraints IEVision60Robot::stateConstraints() const {
  NonlinearFactorGraph graph;
  const auto &values = params->state_constrianed_values;
  for (const auto &key : values.keys()) {
    DynamicsSymbol symb(key);
    if (symb.label() == "q") {
      graph.addPrior<double>(
          key, values.at<double>(key),
          noiseModel::Isotropic::Sigma(1, params->tol_prior_q));
    } else if (symb.label() == "v") {
      graph.addPrior<double>(
          key, values.at<double>(key),
          noiseModel::Isotropic::Sigma(1, params->tol_prior_v));
    } else if (symb.label() == "p") {
      graph.addPrior<Pose3>(
          key, values.at<Pose3>(key),
          noiseModel::Isotropic::Sigma(6, params->tol_prior_q));
    } else if (symb.label() == "V") {
      graph.addPrior<Vector6>(
          key, values.at<Vector6>(key),
          noiseModel::Isotropic::Sigma(6, params->tol_prior_v));
    }
  }
  return ConstraintsFromGraph(graph);
}

/* ************************************************************************* */
InequalityConstraints IEVision60Robot::iConstraints(const size_t k) const {
  InequalityConstraints constraints;
  if (params->include_friction_cone) {
    constraints.add(frictionConeConstraints(k));
  }
  if (params->include_joint_limits) {
    constraints.add(jointLimitConstraints(k));
  }
  if (params->include_torque_limits) {
    constraints.add(torqueLimitConstraints(k));
  }
  if (params->include_collision_free_z) {
    constraints.add(groundCollisionFreeConstraints(k));
  }
  if (params->include_collision_free_s) {
    constraints.add(obstacleCollisionFreeConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::linkCollocationFactors(const uint8_t link_id, const size_t &k,
                                        const double &dt) const {
  NonlinearFactorGraph graph;
  if (params->collocation == CollocationScheme::Trapezoidal) {
    graph.emplace_shared<gtdynamics::FixTimeTrapezoidalPoseCollocationFactor>(
        PoseKey(link_id, k), PoseKey(link_id, k + 1), TwistKey(link_id, k),
        TwistKey(link_id, k + 1), dt, graph_builder.opt().pose_col_cost_model);
    graph.emplace_shared<gtdynamics::FixTimeTrapezoidalTwistCollocationFactor>(
        TwistKey(link_id, k), TwistKey(link_id, k + 1),
        TwistAccelKey(link_id, k), TwistAccelKey(link_id, k + 1), dt,
        graph_builder.opt().twist_col_cost_model);
  } else {
    graph.emplace_shared<gtdynamics::FixTimeEulerPoseCollocationFactor>(
        PoseKey(link_id, k), PoseKey(link_id, k + 1), TwistKey(link_id, k), dt,
        graph_builder.opt().pose_col_cost_model);
    graph.emplace_shared<gtdynamics::FixTimeEulerTwistCollocationFactor>(
        TwistKey(link_id, k), TwistKey(link_id, k + 1),
        TwistAccelKey(link_id, k), dt,
        graph_builder.opt().twist_col_cost_model);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::multiPhaseLinkCollocationFactors(
    const uint8_t link_id, const size_t &k, const Key &phase_key) const {
  NonlinearFactorGraph graph;
  if (params->collocation == CollocationScheme::Trapezoidal) {
    graph.add(gtdynamics::TrapezoidalPoseCollocationFactor(
        PoseKey(link_id, k), PoseKey(link_id, k + 1), TwistKey(link_id, k),
        TwistKey(link_id, k + 1), phase_key,
        graph_builder.opt().pose_col_cost_model));
    graph.add(gtdynamics::TrapezoidalTwistCollocationFactor(
        TwistKey(link_id, k), TwistKey(link_id, k + 1),
        TwistAccelKey(link_id, k), TwistAccelKey(link_id, k + 1), phase_key,
        graph_builder.opt().twist_col_cost_model));
  } else {
    graph.add(gtdynamics::EulerPoseCollocationFactor(
        PoseKey(link_id, k), PoseKey(link_id, k + 1), TwistKey(link_id, k),
        phase_key, graph_builder.opt().pose_col_cost_model));
    graph.add(gtdynamics::EulerTwistCollocationFactor(
        TwistKey(link_id, k), TwistKey(link_id, k + 1),
        TwistAccelKey(link_id, k), phase_key,
        graph_builder.opt().twist_col_cost_model));
  }

  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::collocationCostsStep(const size_t k, const double dt) const {
  NonlinearFactorGraph graph;
  graph.add(linkCollocationFactors(base_id, k, dt));

  // TODO: add version for Euler
  for (size_t leg_idx = 0; leg_idx < 4; leg_idx++) {
    if (!phase_info->contact_indices.exists(leg_idx)) {
      for (const auto &joint : legs.at(leg_idx).joints) {
        uint8_t j = joint->id();
        Key q0_key = JointAngleKey(j, k);
        Key q1_key = JointAngleKey(j, k + 1);
        Key v0_key = JointVelKey(j, k);
        Key v1_key = JointVelKey(j, k + 1);
        Key a0_key = JointAccelKey(j, k);
        Key a1_key = JointAccelKey(j, k + 1);
        DynamicsGraph::addCollocationFactorDouble(
            &graph, q0_key, q1_key, v0_key, v1_key, dt,
            graph_builder.opt().q_col_cost_model, params->collocation);
        DynamicsGraph::addCollocationFactorDouble(
            &graph, v0_key, v1_key, a0_key, a1_key, dt,
            graph_builder.opt().v_col_cost_model, params->collocation);
      }
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::collocationCosts(const size_t num_steps,
                                                       double dt) const {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < num_steps; k++) {
    graph.add(collocationCostsStep(k, dt));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::multiPhaseCollocationCostsStep(const size_t k,
                                                const size_t phase_id) const {
  NonlinearFactorGraph graph;
  Key phase_key = PhaseKey(phase_id);
  graph.add(multiPhaseLinkCollocationFactors(base_id, k, phase_key));

  for (size_t leg_idx = 0; leg_idx < 4; leg_idx++) {
    if (!phase_info->contact_indices.exists(leg_idx)) {
      for (const auto &joint : legs.at(leg_idx).joints) {
        uint8_t j = joint->id();
        Key q0_key = JointAngleKey(j, k);
        Key q1_key = JointAngleKey(j, k + 1);
        Key v0_key = JointVelKey(j, k);
        Key v1_key = JointVelKey(j, k + 1);
        Key a0_key = JointAccelKey(j, k);
        Key a1_key = JointAccelKey(j, k + 1);
        DynamicsGraph::addMultiPhaseCollocationFactorDouble(
            &graph, q0_key, q1_key, v0_key, v1_key, phase_key,
            graph_builder.opt().q_col_cost_model, params->collocation);
        DynamicsGraph::addMultiPhaseCollocationFactorDouble(
            &graph, v0_key, v1_key, a0_key, a1_key, phase_key,
            graph_builder.opt().v_col_cost_model, params->collocation);
      }
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::multiPhaseCollocationCosts(const size_t start_step,
                                            const size_t end_step,
                                            const size_t phase_id) const {
  NonlinearFactorGraph graph;
  for (size_t k = start_step; k < end_step; k++) {
    graph.add(multiPhaseCollocationCostsStep(k, phase_id));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::actuationRmseTorqueCosts(const size_t k) const {
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    graph.add(
        gtdynamics::MinTorqueFactor(TorqueKey(joint->id(), k), actuation_nm));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::actuationImpulseCosts(const size_t k, const size_t phase_idx,
                                       bool apply_sqrt) const {
  // TODO: add option with apply_sqrt
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    Double_ torque_curr(TorqueKey(joint->id(), k));
    Double_ torque_next(TorqueKey(joint->id(), k + 1));
    Double_ phase_dt(PhaseKey(phase_idx));
    Double_ impulse = 0.5 * (torque_curr + torque_next) * phase_dt;
    graph.emplace_shared<ExpressionFactor<double>>(actuation_nm, 0.0, impulse);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::actuationWorkCosts(const size_t k, bool apply_sqrt) const {
  // TODO: distinguish positive work and negative work
  NonlinearFactorGraph graph;
  for (auto &&joint : robot.joints()) {
    Double_ torque_curr(TorqueKey(joint->id(), k));
    Double_ torque_next(TorqueKey(joint->id(), k + 1));
    Double_ q_curr(JointAngleKey(joint->id(), k));
    Double_ q_next(JointAngleKey(joint->id(), k + 1));
    Double_ work = 0.5 * (torque_curr + torque_next) * (q_next - q_curr);
    graph.emplace_shared<ExpressionFactor<double>>(actuation_nm, 0.0, work);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::actuationCosts(const size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k <= num_steps; k++) {
    graph.add(actuationRmseTorqueCosts(k));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::jerkCosts(const size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < num_steps; k++) {
    for (auto &&joint : robot.joints())
      graph.emplace_shared<BetweenFactor<double>>(TorqueKey(joint->id(), k),
                                                  TorqueKey(joint->id(), k + 1),
                                                  0.0, jerk_nm);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::accelPenaltyCosts(const size_t num_steps) const {
  NonlinearFactorGraph graph;

  double accel_penalty_threshold = params->accel_panalty_threshold;
  auto penalty_func =
      [accel_penalty_threshold](const double &a,
                                gtsam::OptionalJacobian<1, 1> H = {}) {
        int sign_a = (a > 0) - (a < 0);
        double result = abs(a) - accel_penalty_threshold;
        if (result < 0) {
          if (H) {
            H->setConstant(0);
          }
          result = 0;
        } else {
          if (H) {
            H->setConstant(sign_a);
          }
        }
        return result;
      };

  for (size_t k = 0; k <= num_steps; k++) {
    for (auto &&joint : robot.joints()) {
      Double_ accel(JointAccelKey(joint->id(), k));
      Double_ accel_penalty_expr(penalty_func, accel);
      graph.emplace_shared<ExpressionFactor<double>>(
          noiseModel::Isotropic::Sigma(1, params->sigma_a_penalty), 0.0,
          accel_penalty_expr);
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::stateCosts() const {
  NonlinearFactorGraph graph;
  const auto &values = params->state_cost_values;
  for (const auto &key : values.keys()) {
    DynamicsSymbol symb(key);
    if (symb.label() == "q") {
      graph.addPrior<double>(key, values.at<double>(key), des_q_nm);
    } else if (symb.label() == "v") {
      graph.addPrior<double>(key, values.at<double>(key), des_v_nm);
    } else if (symb.label() == "p") {
      graph.addPrior<Pose3>(key, values.at<Pose3>(key), des_pose_nm);
    } else if (symb.label() == "V") {
      graph.addPrior<Vector6>(key, values.at<Vector6>(key), des_pose_nm);
    }
  }
  for (const auto &[i, point_l, point_w, k] : params->state_cost_points) {
    graph.add(statePointCostFactor(i, point_l, point_w, k));
  }
  for (const auto &[i, point_l, vel_w, k] : params->state_cost_point_vels) {
    graph.add(statePointVelCostFactor(i, point_l, vel_w, k));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60Robot::symmetryCosts(const size_t k) const {
  NonlinearFactorGraph graph;
  for (const auto &joint : robot.joints()) {
    if (isLeft(joint->name())) {
      std::string counterpart_name = counterpart(joint->name());
      size_t left_id = joint->id();
      size_t right_id = robot.joint(counterpart_name)->id();
      if (!isHip(joint->name())) {
        graph.emplace_shared<BetweenFactor<double>>(JointAngleKey(left_id, k),
                                                    JointAngleKey(right_id, k),
                                                    0.0, symmetry_nm);
      }
      else {
        Double_ left_q(JointAngleKey(left_id, k));
        Double_ right_q(JointAngleKey(right_id, k));
        Double_ error = left_q + right_q;
        graph.emplace_shared<ExpressionFactor<double>>(symmetry_nm, 0.0, error);
      }
    }
  }
  return graph;
}

} // namespace gtsam
