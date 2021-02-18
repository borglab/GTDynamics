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

#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/**
 * @class FixedJoint class represents a fixed joint and contains all necessary
 * factor construction methods.
 */
class FixedJoint : public JointTyped {
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
};

}  // namespace gtdynamics
