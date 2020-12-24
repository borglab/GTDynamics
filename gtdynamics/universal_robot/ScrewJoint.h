/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJoint.h
 * @brief Representation of screw joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_

#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @class ScrewJoint is an implementation of the ScrewJointBase class
 *  which represents a screw joint and contains all necessary factor
 *  construction methods.
 */
class ScrewJoint : public ScrewJointBase {
 protected:
  /// Returns the screw axis in the joint frame given the joint axis and thread
  /// pitch
  gtsam::Vector6 getScrewAxis(const gtsam::Vector3 &axis, double thread_pitch) {
    gtsam::Vector6 screw_axis;
    screw_axis << axis, axis * thread_pitch / 2 / M_PI;
    return screw_axis;
  }

 public:
  /**
   * @brief Create ScrewJoint using Parameters, joint name, joint pose in
   * world frame, screw axes, and parent and child links.
   *
   * @param[in] name          Name of the joint
   * @param[in] wTj           joint pose expressed in world frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   * @param[in] parameters    Joint::Parameters struct
   * @param[in] axis          joint axis expressed in joint frame
   * @param[in] thread_pitch  joint's thread pitch in dist per rev
   */
  ScrewJoint(const std::string &name, const gtsam::Pose3 &wTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
             const Parameters &parameters, const gtsam::Vector3 &axis,
             double thread_pitch)
      : ScrewJointBase(name, wTj, parent_link, child_link, parameters, axis,
                       getScrewAxis(axis, thread_pitch)) {}

  /// Return joint type for use in reconstructing robot from Parameters.
  Type type() const override { return Type::Screw; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_
