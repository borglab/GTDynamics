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
   * @brief Create PrismaticJoint from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                       sdf::Joint object.
   * @param[in] effort_type                     Joint effort type.
   * @param[in] spring_coefficient              Spring coefficient.
   * @param[in] joint_limit_threshold           Joint angle limit threshold.
   * @param[in] velocity_limit_threshold        Joint velocity limit threshold.
   * @param[in] acceleration_limit              Joint acceleration limit.
   * @param[in] acceleration_limit_threshold    Joint Acceleration limit threshold.
   * @param[in] torque_limit_threshold          Joint torque limit threshold.
   * @param[in] parent_link                     Shared pointer to the parent Link.
   * @param[in] child_link                      Shared pointer to the child Link.
   */
  PrismaticJoint(const sdf::Joint &sdf_joint,
                JointEffortType effort_type,
                double spring_coefficient, double joint_limit_threshold,
                double velocity_limit_threshold, double acceleration_limit,
                double acceleration_limit_threshold, double torque_limit_threshold,
                LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJointBase(sdf_joint,
                       getScrewAxis(getSdfAxis(sdf_joint)),
                       effort_type, spring_coefficient,
                       joint_limit_threshold, velocity_limit_threshold,
                       acceleration_limit, acceleration_limit_threshold,
                       torque_limit_threshold, parent_link, child_link) {}

  /** 
   * @brief Create PrismaticJoint using sdf::Joint instance and joint parameters. 
   * 
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] parameters                 ScrewJointBase::Params struct.
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
  */
  PrismaticJoint(const sdf::Joint &sdf_joint,
                 const Params &parameters,
                 LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : PrismaticJoint(
          sdf_joint,
          parameters.effort_type, parameters.spring_coefficient,
          parameters.joint_limit_threshold, parameters.velocity_limit_threshold,
          parameters.acceleration_limit, parameters.acceleration_limit_threshold,
          parameters.torque_limit_threshold, parent_link, child_link) {}

  /** 
   * @brief Create PrismaticJoint using Params, joint name, joint pose in 
   * world frame, screw axes, and parent and child links.
   * 
   * @param[in] params        ScrewJointBase::Params struct
   * @param[in] name          Name of the joint
   * @param[in] wTj           joint pose expressed in world frame
   * @param[in] axis          joint axis expressed in joint frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
  */
  PrismaticJoint(const Params &params, const std::string &name, 
                 const gtsam::Pose3 &wTj, gtsam::Vector3 axis, 
                 LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJointBase(params, name, wTj, axis, getScrewAxis(axis),
                       parent_link, child_link) {}

  /// Return jointType for use in reconstructing robot from Parameters.
  JointType jointType() const { return JointType::Prismatic; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_
