#pragma once

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {
boost::shared_ptr<const ScrewJointBase> make_joint(gtsam::Pose3 cMp,
                                                   gtsam::Vector6 cScrewAxis) {
  // create links
  LinkParams link1_params, link2_params;
  link1_params.mass = 100;
  link1_params.name = "l1";
  link1_params.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  link1_params.wTl = gtsam::Pose3();
  link1_params.lTcom = gtsam::Pose3();
  link2_params = link1_params;
  link2_params.wTl = cMp.inverse();

  LinkSharedPtr l1 = boost::make_shared<Link>(Link(link1_params));
  LinkSharedPtr l2 = boost::make_shared<Link>(Link(link2_params));

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
      1, "j1", wTj, l1, l2, joint_params, jScrewAxis.head<3>(), jScrewAxis));
}
} // namespace gtdynamics