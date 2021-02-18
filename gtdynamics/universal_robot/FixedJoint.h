/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  FixedJoint.h
 * @brief Representation of revolute joint.
 * @author Varun Agrawal
 */

#pragma once

#include "gtdynamics/factors/JointLimitFactor.h"
#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/**
 * @class FixedJoint class represents a fixed joint and contains all necessary
 * factor construction methods.
 */
class FixedJoint : public JointTyped {
  using Pose3 = gtsam::Pose3;

 public:
  /**
   * @brief Create FixedJoint using Parameters, joint name, joint pose in
   * world frame, and parent and child links.
   *
   * @param[in] name          Name of the joint
   * @param[in] wTj           joint pose expressed in world frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   * @param[in] parameters    Joint::Parameters struct
   * @param[in] axis          joint axis expressed in joint frame
   */
  FixedJoint(const std::string &name, const gtsam::Pose3 &wTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
             const Parameters &parameters)
      : JointTyped(name, wTj, parent_link, child_link, parameters) {}

  /// Return joint type for use in reconstructing robot from Parameters.
  Type type() const override { return Type::Fixed; }

  // inherit overloads
  using JointTyped::transformTo;
  using JointTyped::transformTwistAccelTo;
  using JointTyped::transformTwistTo;

  /// Return the transform from the other link com to this link com frame
  Pose3 transformTo(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none) const override {
    return parentLink()->wTl();
  }

  /**
   * Return the twist of this link given the other link's twist and joint angle.
   */
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      boost::optional<double> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist =
          boost::none) const override {
    // TODO Temporary implementation. Need to redo this correctly.
    gtsam::Vector6 other_twist_ =
        other_twist ? *other_twist : gtsam::Vector6::Zero();

    if (H_q) {
      *H_q = gtsam::Vector6::Zero();
    }
    if (H_q_dot) {
      *H_q_dot = gtsam::Vector6::Zero();
    }
    if (H_other_twist) {
      *H_other_twist = gtsam::Matrix66();
    }

    return other_twist_;
  }

  /**
   * Return the twist acceleration of this link given the other link's twist
   * acceleration, twist, and joint angle and this link's twist.
   */
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      boost::optional<double> q_dot = boost::none,
      boost::optional<double> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const override {
    // TODO Temporary implementation. Need to redo this correctly.
    gtsam::Vector6 other_twist_accel_ =
        other_twist_accel ? *other_twist_accel : gtsam::Vector6::Zero();

    // i = other link
    // j = this link
    gtsam::Pose3 jTi = transformTo(link, q);

    gtsam::Vector6 this_twist_accel = other_twist_accel_;

    if (H_other_twist_accel) {
      *H_other_twist_accel = jTi.AdjointMap();
    }
    if (H_q) {
      *H_q = gtsam::Vector6::Zero();
    }
    if (H_q_dot) {
      *H_q_dot = gtsam::Vector6::Zero();
    }
    if (H_q_ddot) {
      *H_q_ddot = gtsam::Vector6::Zero();
    }

    return this_twist_accel;
  }

  /// Return joint limit factors.
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const override {
    // TODO Temporary implementation. Need to redo this correctly.
    gtsam::NonlinearFactorGraph graph;
    auto id = getID();
    // Add joint angle limit factor.
    graph.emplace_shared<JointLimitFactor>(
        JointAngleKey(id, t), opt.jl_cost_model,
        parameters().scalar_limits.value_lower_limit,
        parameters().scalar_limits.value_upper_limit,
        parameters().scalar_limits.value_limit_threshold);

    // Add joint velocity limit factors.
    graph.emplace_shared<JointLimitFactor>(
        JointVelKey(id, t), opt.jl_cost_model, -parameters().velocity_limit,
        parameters().velocity_limit, parameters().velocity_limit_threshold);

    // Add joint acceleration limit factors.
    graph.emplace_shared<JointLimitFactor>(
        JointAccelKey(id, t), opt.jl_cost_model,
        -parameters().acceleration_limit, parameters().acceleration_limit,
        parameters().acceleration_limit_threshold);

    // Add joint torque limit factors.
    graph.emplace_shared<JointLimitFactor>(
        TorqueKey(id, t), opt.jl_cost_model, -parameters().torque_limit,
        parameters().torque_limit, parameters().torque_limit_threshold);
    return graph;
  }

  JointTorque transformWrenchToTorque(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<1, 6> H_wrench = boost::none) const override {
    // TODO Temporary implementation. Need to redo this correctly.
    return JointTorque();
  }

  gtsam::Matrix6 AdjointMapJacobianJointAngle(
      const LinkSharedPtr &link,
      boost::optional<double> q = boost::none) const override {
    // TODO Temporary implementation. Need to redo this correctly.
    return gtsam::Matrix6();
  }
};

}  // namespace gtdynamics
