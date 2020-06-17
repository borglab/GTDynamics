/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PrismaticJoint.h
 * @brief Representation of PrismaticJoint that inherits from ScrewJointBase
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_

#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @class PrismaticJoint is an implementation of the ScrewJointBase class
 *  which represents a prismatic joint and contains all necessary factor
 *  construction methods.
 */
class PrismaticJoint : public ScrewJointBase {
 protected:
  /// Returns the screw axis in the joint frame given the joint axis
  gtsam::Vector6 getScrewAxis(gtsam::Vector3 axis) {
    gtsam::Vector6 screw_axis;
    screw_axis << 0, 0, 0, axis;
    return screw_axis;
  }

 public:
  /** 
   * @brief Create PrismaticJoint using Parameters, joint name, joint pose in 
   * world frame, screw axes, and parent and child links.
   * 
   * @param[in] name          Name of the joint
   * @param[in] wTj           joint pose expressed in world frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   * @param[in] parameters    ScrewJointBase::Parameters struct
   * @param[in] axis          joint axis expressed in joint frame
  */
  PrismaticJoint(const std::string &name, const gtsam::Pose3 &wTj,
                 LinkSharedPtr parent_link, LinkSharedPtr child_link,
                 const Parameters &parameters, gtsam::Vector3 axis)
      : ScrewJointBase(name, wTj, parent_link, child_link, parameters, axis,
                       getScrewAxis(axis)) {}

  /**
   * @brief Create PrismaticJoint from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                       sdf::Joint object.
   * @param[in] parent_link                     Shared pointer to the parent Link.
   * @param[in] child_link                      Shared pointer to the child Link.
   */
  PrismaticJoint(const sdf::Joint &sdf_joint, LinkSharedPtr parent_link,
                 LinkSharedPtr child_link)
      : ScrewJointBase(sdf_joint, parent_link, child_link,
                       getScrewAxis(getSdfAxis(sdf_joint))) {}

  /** 
   * @brief Create PrismaticJoint using sdf::Joint instance and joint parameters. 
   * 
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
   * @param[in] parameters                 ScrewJointBase::Parameters struct.
  */
  PrismaticJoint(const sdf::Joint &sdf_joint, LinkSharedPtr parent_link,
                 LinkSharedPtr child_link, const Parameters &parameters)
      : ScrewJointBase(sdf_joint, parent_link, child_link, parameters,
                       getScrewAxis(getSdfAxis(sdf_joint))) {}

  /// Return jointType for use in reconstructing robot from Parameters.
  JointType jointType() const { return JointType::Prismatic; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_
