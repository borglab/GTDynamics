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
  gtsam::Vector6 getScrewAxis(gtsam::Vector3 axis, double thread_pitch) {
    gtsam::Vector6 screw_axis;
    screw_axis << axis, axis * thread_pitch / 2 / M_PI;
    return screw_axis;
  }

 public:
  /**
   * @brief Create ScrewJoint from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                      sdf::Joint object.
   * @param[in] effort_type                    Joint effort type.
   * @param[in] spring_coefficient             Spring coefficient.
   * @param[in] joint_limit_threshold          Joint angle limit threshold.
   * @param[in] velocity_limit_threshold       Joint velocity limit threshold.
   * @param[in] acceleration_limit             Joint acceleration limit.
   * @param[in] acceleration_limit_threshold   Joint Acceleration limit threshold.
   * @param[in] torque_limit_threshold         Joint torque limit threshold.
   * @param[in] parent_link                    Shared pointer to the parent Link.
   * @param[in] child_link                     Shared pointer to the child Link.
   */
  ScrewJoint(const sdf::Joint &sdf_joint,
                JointEffortType effort_type,
                double spring_coefficient, double joint_limit_threshold,
                double velocity_limit_threshold, double acceleration_limit,
                double acceleration_limit_threshold, double torque_limit_threshold,
                LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJointBase(sdf_joint,
                       getScrewAxis(getSdfAxis(sdf_joint),
                                    sdf_joint.ThreadPitch()),
                       effort_type, spring_coefficient,
                       joint_limit_threshold, velocity_limit_threshold,
                       acceleration_limit, acceleration_limit_threshold,
                       torque_limit_threshold, parent_link, child_link) {}

  /** 
   * @brief Create ScrewJoint using sdf::Joint instance and joint parameters. 
   * 
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] parameters                 Joint::Params struct
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
  */
  ScrewJoint(const sdf::Joint &sdf_joint,
                 const Params &parameters,
                 LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJoint(
          sdf_joint,
          parameters.effort_type, parameters.spring_coefficient,
          parameters.joint_limit_threshold, parameters.velocity_limit_threshold,
          parameters.acceleration_limit, parameters.acceleration_limit_threshold,
          parameters.torque_limit_threshold, parent_link, child_link) {}

  /** 
   * @brief Create ScrewJoint using Params and screw axes.
   * 
   * @param[in] params        Joint::Params struct
   * @param[in] axis          joint axis expressed in joint frame
   * @param[in] thread_pitch  joint's thread pitch in dist per rev
  */
  ScrewJoint(const Params &params, gtsam::Vector3 axis, double thread_pitch)
      : ScrewJointBase(params,
                       axis,
                       getScrewAxis(axis, thread_pitch)) {}

  /// Return jointType for use in reconstructing robot from Parameters.
  JointType jointType() const { return JointType::Screw; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_
