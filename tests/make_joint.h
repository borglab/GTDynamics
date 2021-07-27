/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  make_joint.h
 * @brief Function often used in tests
 * @author: Frank Dellaert
 */

#pragma once

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {
/// Create a joint with given rest transform cMp and screw-axis in child frame.
boost::shared_ptr<const ScrewJointBase> make_joint(gtsam::Pose3 cMp,
                                                   gtsam::Vector6 cScrewAxis) {
  // create links
  std::string name = "l1";
  double mass = 100;
  gtsam::Matrix3 inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  gtsam::Pose3 wTl, lTcom;

  auto l1 = boost::make_shared<Link>(Link(1, name, mass, inertia, lTcom));
  auto l2 = boost::make_shared<Link>(
      Link(2, name, mass, inertia, cMp.inverse(), lTcom));

  // create joint
  JointParams joint_params;
  joint_params.effort_type = JointEffortType::Actuated;
  joint_params.scalar_limits.value_lower_limit = -1.57;
  joint_params.scalar_limits.value_upper_limit = 1.57;
  joint_params.scalar_limits.value_limit_threshold = 0;
  gtsam::Pose3 wTj = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 jTccom = wTj.inverse() * l2->wTcom();
  gtsam::Vector6 jScrewAxis = jTccom.AdjointMap() * cScrewAxis;

  return boost::make_shared<const ScrewJointBase>(ScrewJointBase(
      1, "j1", l1, l2, jScrewAxis.head<3>(), jScrewAxis, joint_params, wTj));
}
}  // namespace gtdynamics
