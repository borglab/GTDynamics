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
#include "utils/values.h"
#include <_types/_uint8_t.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactPointFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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
    if (params.express_contact_force) {
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

  if (params.express_redundancy) {
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

        if (params.express_contact_force) {
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
                             params.gravity));
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
  NonlinearFactorGraph graph = graph_builder.aFactors(robot, t, contact_points);
  graph.add(DynamicsFactors(t));
  if (contact_link_ids.size() == 4) {
    graph.add(contactRedundancyFactor(t));
  }

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
  for (const auto &it : params.joint_lower_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key joint_key = JointAngleKey(joint_id, k);
    Double_ q_expr(joint_key);
    Double_ q_min_expr = q_expr - Double_(it.second);
    constraints.emplace_shared<DoubleExpressionInequality>(q_min_expr,
                                                           params.tol_jl);
  }
  for (const auto &it : params.joint_upper_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key joint_key = JointAngleKey(joint_id, k);
    Double_ q_expr(joint_key);
    Double_ q_max_expr = Double_(it.second) - q_expr;
    constraints.emplace_shared<DoubleExpressionInequality>(q_max_expr,
                                                           params.tol_jl);
  }
  return constraints;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
IEVision60Robot::torqueLimitConstraints(const size_t k) const {
  InequalityConstraints constraints;
  for (const auto &it : params.torque_lower_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key torque_key = TorqueKey(joint_id, k);
    Double_ tau_expr(torque_key);
    Double_ tau_min_expr = tau_expr - Double_(it.second);
    constraints.emplace_shared<DoubleExpressionInequality>(tau_min_expr,
                                                           params.tol_tl);
  }
  for (const auto &it : params.torque_upper_limits) {
    size_t joint_id = robot.joint(it.first)->id();
    Key torque_key = TorqueKey(joint_id, k);
    Double_ tau_expr(torque_key);
    Double_ tau_max_expr = Double_(it.second) - tau_expr;
    constraints.emplace_shared<DoubleExpressionInequality>(tau_max_expr,
                                                           params.tol_tl);
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
  if (params.include_friction_cone) {
    constraints.add(frictionConeConstraints(k));
  }
  if (params.include_joint_limits) {
    constraints.add(jointLimitConstraints(k));
  }
  if (params.include_torque_limits) {
    constraints.add(torqueLimitConstraints(k));
  }
  if (params.include_collision_avoidance) {
    constraints.add(collisionAvoidanceConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60Robot::linkCollocationFactors(const uint8_t link_id, const size_t &k,
                                        const double &dt) const {
  NonlinearFactorGraph graph;
  if (params.collocation == CollocationScheme::Trapezoidal) {
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
  if (params.collocation == CollocationScheme::Trapezoidal) {
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
    if (!params.contact_indices.exists(leg_idx)) {
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
            graph_builder.opt().q_col_cost_model, params.collocation);
        DynamicsGraph::addCollocationFactorDouble(
            &graph, v0_key, v1_key, a0_key, a1_key, dt,
            graph_builder.opt().v_col_cost_model, params.collocation);
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
  graph.add(multiPhaseLinkCollocationFactors(base_id, k, phase_key));

  graph.add(multiPhaseLinkCollocationFactors(base_id, k, phase_key));

  for (size_t leg_idx = 0; leg_idx < 4; leg_idx++) {
    if (!params.contact_indices.exists(leg_idx)) {
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
            graph_builder.opt().q_col_cost_model, params.collocation);
        DynamicsGraph::addMultiPhaseCollocationFactorDouble(
            &graph, v0_key, v1_key, a0_key, a1_key, phase_key,
            graph_builder.opt().v_col_cost_model, params.collocation);
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
NonlinearFactorGraph IEVision60Robot::stateCosts(
    const Values &values, const std::optional<KeyVector> &optional_keys) const {
  NonlinearFactorGraph graph;
  KeyVector keys = optional_keys ? *optional_keys : values.keys();
  for (const auto &key : keys) {
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
  return graph;
}

} // namespace gtsam
