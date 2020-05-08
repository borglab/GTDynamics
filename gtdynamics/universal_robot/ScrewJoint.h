/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJoint.h
 * @brief Representation of revolute joint.
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
 *  This uses the Curiously Recurring Template Pattern (CRTP) for static
 *  polymorphism and implements the setScrewAxis() and jointType() functions.
 */
class ScrewJoint : public ScrewJointBase<ScrewJoint> {
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
                       (gtsam::Vector6() << sdf_joint.Axis()->Xyz()[0],
                                            sdf_joint.Axis()->Xyz()[1],
                                            sdf_joint.Axis()->Xyz()[2],
          sdf_joint.Axis()->Xyz()[0] * sdf_joint.ThreadPitch() / 2 / M_PI, // normalize Axis ?
          sdf_joint.Axis()->Xyz()[1] * sdf_joint.ThreadPitch() / 2 / M_PI,
          sdf_joint.Axis()->Xyz()[2] * sdf_joint.ThreadPitch() / 2 / M_PI).finished(),
                       joint_effort_type, springCoefficient,
                       jointLimitThreshold, velocityLimitThreshold,
                       accelerationLimit, accelerationLimitThreshold,
                       torqueLimitThreshold, parent_link, child_link) {}

  /** Construct joint using sdf::Joint instance and joint parameters. */
  ScrewJoint(const sdf::Joint &sdf_joint,
                 const gtdynamics::JointParams &jps,
                 LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : ScrewJoint(
          sdf_joint,
          jps.jointEffortType, jps.springCoefficient,
          jps.jointLimitThreshold, jps.velocityLimitThreshold,
          jps.accelerationLimit, jps.accelerationLimitThreshold,
          jps.torqueLimitThreshold, parent_link, child_link) {}

  /** constructor using JointParams and screw axes */
  explicit ScrewJoint(const Params &params)
      : ScrewJointBase(params,
                       (gtsam::Vector6() << params.axis,
          params.axis * params.thread_pitch / 2 / M_PI).finished()) {}

  /// Return jointType
  char jointType() const { return 'H'; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_