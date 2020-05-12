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
class ScrewJoint : public ScrewJointBase<ScrewJoint> {
 protected:
  /// Returns the screw axis in the joint frame given the joint axis and thread
  /// pitch
  gtsam::Vector6 getScrewAxis(gtsam::Vector3 axis, double thread_pitch) {
    gtsam::Vector6 screw_axis;
    screw_axis << axis, axis * thread_pitch / 2 / M_PI;
    return screw_axis;
  }

 public:
  /**
   * @brief Create ScrewJoint from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] joint_effort_type          Joint effort type.
   * @param[in] springCoefficient          Spring coefficient.
   * @param[in] jointLimitThreshold        Joint angle limit threshold.
   * @param[in] velocityLimitThreshold     Joint velocity limit threshold.
   * @param[in] accelerationLimit          Joint acceleration limit.
   * @param[in] accelerationLimitThreshold Joint Acceleration limit threshold.
   * @param[in] torqueLimitThreshold       Joint torque limit threshold.
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
   */
  ScrewJoint(const sdf::Joint &sdf_joint,
                JointEffortType joint_effort_type,
                double springCoefficient, double jointLimitThreshold,
                double velocityLimitThreshold, double accelerationLimit,
                double accelerationLimitThreshold, double torqueLimitThreshold,
                LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJointBase(sdf_joint,
                       getScrewAxis(Joint::getSdfAxis(sdf_joint),
                                    sdf_joint.ThreadPitch()),
                       joint_effort_type, springCoefficient,
                       jointLimitThreshold, velocityLimitThreshold,
                       accelerationLimit, accelerationLimitThreshold,
                       torqueLimitThreshold, parent_link, child_link) {}

  /** 
   * @brief Create ScrewJoint using sdf::Joint instance and joint parameters. 
   * 
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] parameters                 Joint::Params struct
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
  */
  ScrewJoint(const sdf::Joint &sdf_joint,
                 const gtdynamics::JointParams &parameters,
                 LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJoint(
          sdf_joint,
          parameters.jointEffortType, parameters.springCoefficient,
          parameters.jointLimitThreshold, parameters.velocityLimitThreshold,
          parameters.accelerationLimit, parameters.accelerationLimitThreshold,
          parameters.torqueLimitThreshold, parent_link, child_link) {}

  /** 
   * @brief Create ScrewJoint using JointParams and screw axes.
   * 
   * @param[in] params        Joint::Params struct
   * @param[in] axis          joint axis expressed in joint frame
   * @param[in] thread_pitch  joint's thread pitch in dist per rev
  */
  ScrewJoint(const Params &params, gtsam::Vector3 axis, double thread_pitch)
      : ScrewJointBase(params,
                       axis,
                       getScrewAxis(axis, thread_pitch)) {}

  /// Return jointType
  char jointType() const { return 'H'; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_
