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
class FixedJoint : public Joint {
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
   * base frame, and parent and child links.
   *
   * @param[in] id            id for keys
   * @param[in] name          Name of the joint
   * @param[in] bTj           joint pose expressed in the base frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   * @param[in] jScrewAxis   Screw axis in the joint frame
   */
  FixedJoint(unsigned char id, const std::string &name, const gtsam::Pose3 &bTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
             const Vector6 &jScrewAxis)
      : Joint(id, name, bTj, parent_link, child_link, jScrewAxis,
              fixedJointParams()) {}

  /// Return joint type for use in reconstructing robot from JointParams.
  Type type() const override { return Type::Fixed; }
};

}  // namespace gtdynamics
