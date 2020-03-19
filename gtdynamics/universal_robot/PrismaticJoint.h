/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PrismaticJoint.h
 * @brief Representation of a prismatic robot joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATIC_JOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATIC_JOINT_H_

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {
  class PrismaticJoint : public Joint {

public:
  /**
   * Create Joint from a sdf::Joint instance, as described in
   * ROS/urdfdom_headers:
   * https://bitbucket.org/osrf/sdformat/src/7_to_gz11/include/sdf/Joint.hh
   *
   * Keyword arguments:
   *   sdf_joint                  -- sdf::Joint instance to derive joint
   * attributes from. jointEffortType_           -- joint effort type.
   *   springCoefficient          -- spring coefficient for Impedence joint.
   *   jointLimitThreshold        -- joint angle limit threshold.
   *   velocityLimitThreshold     -- joint velocity limit threshold.
   *   accelerationLimit          -- joint acceleration limit
   *   accelerationLimitThreshold -- joint acceleration limit threshold
   *   torqueLimitThreshold       -- joint torque limit threshold
   *   parent_link                -- shared pointer to the parent Link.
   *   child_link                 -- shared pointer to the child Link.
   */
  PrismaticJoint(const sdf::Joint &sdf_joint, JointEffortType joint_effort_type,
        double springCoefficient, double jointLimitThreshold,
        double velocityLimitThreshold, double accelerationLimit,
        double accelerationLimitThreshold, double torqueLimitThreshold,
        LinkSharedPtr parent_link, LinkSharedPtr child_link) : Joint(
            sdf_joint, joint_effort_type, springCoefficient, jointLimitThreshold, velocityLimitThreshold, accelerationLimit,
            accelerationLimitThreshold, torqueLimitThreshold, parent_link, child_link) {
    setScrewAxis();
  }

  /** constructor using JointParams */
  explicit PrismaticJoint(const Params &params)
      : Joint (params) {
    setScrewAxis();
  }

private:
  void setScrewAxis() {
    jTpcom_ = wTj_.inverse() * parent_link_->wTcom();
    jTccom_ = wTj_.inverse() * child_link_->wTcom();

    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    pScrewAxis_ << 0, 0, 0, pcomRj * -axis_;
    cScrewAxis_ << 0, 0, 0, ccomRj * axis_;
  }

  };
}


#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATIC_JOINT_H_