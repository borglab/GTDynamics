/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  FixedJoint.h
 * @brief Representation of a fixed joint.
 * @author Varun Agrawal
 */

#pragma once

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {

/**
 * @class FixedJoint is a joint which has 0 degrees of freedom.
 */
class FixedJoint : public JointTyped {
 private:
  JointParams fixedJointParams() {
    JointParams params;
    params.effort_type = JointEffortType::Unactuated;
    params.velocity_limit = 0.0;
    params.acceleration_limit = 0.0;
    params.torque_limit = 0.0;
    return params;
  }

 public:
  /**
   * @brief Create FixedJoint using joint name, joint pose in
   * world frame, and parent and child links.
   *
   * @param[in] id            id for keys
   * @param[in] name          Name of the joint
   * @param[in] wTj           joint pose expressed in world frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   */
  FixedJoint(unsigned char id, const std::string &name, const gtsam::Pose3 &wTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link)
      : Joint(id, name, wTj, parent_link, child_link, fixedJointParams()) {}

  /// Return joint type for use in reconstructing robot from JointParams.
  Type type() const override { return Type::Fixed; }

  Pose3 parentTchild(
      JointCoordinate q, size_t t = 0,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none) const override {
    if (H_q) {
      *H_q = gtsam::Z_6x1;
    }
    return pMccom_;
  }

  Pose3 childTparent(
      JointCoordinate q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    if (H_q) {
      *H_q = gtsam::Z_6x1;
    }
    return pMccom_.inverse();
  }

  gtsam::Vector6 transformTwistTo(
      size_t t, const LinkSharedPtr &link, const gtsam::Values &q_and_q_dot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist =
          boost::none) const override {
    if(H_q) {
      *H_q = gtsam::Z_6x1;
    }
    if(H_q_dot) {
      *H_q_dot = gtsam::Z_6x1;
    }
    if(H_other_twist) {
      *H_other_twist = gtsam::I_6x6;
    }
    return gtsam::Z_6x1;
  }

  gtsam::Vector6 transformTwistAccelTo(
      size_t t, const LinkSharedPtr &link,
      const gtsam::Values &q_and_q_dot_and_q_ddot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const override {
    return gtsam::Z_6x1;
  }

  /// Joint-induced twist in child frame
  gtsam::Vector6 childTwist(const gtsam::Values &values,
                            size_t t = 0) const override {
    return gtsam::Z_6x1;
  }

  /// Joint-induced twist in parent frame
  gtsam::Vector6 parentTwist(const gtsam::Values &values,
                             size_t t = 0) const override {
    return gtsam::Z_6x1;
  }
};

}  // namespace gtdynamics
