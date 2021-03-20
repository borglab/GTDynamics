/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  values.cpp
 * @brief Shortcots for inserting/retrieving variables in/from gtsam::Values.
 * @author: Frank Dellaert
 * @author: Varun Agrawal
 */

#include <gtdynamics/utils/values.h>

namespace gtdynamics {

gtsam::Vector JointAngle(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointAngleKey(j, t));
}

gtsam::Vector JointVel(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointVelKey(j, t));
}

gtsam::Vector JointAccel(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointAccelKey(j, t));
}

/// Retrieve torque on the j-th joint at time t.
gtsam::Vector Torque(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::TorqueKey(j, t));
};

/// Insert pose for i-th link at time t.
void InsertPose(gtsam::Values *values, int i, int t, gtsam::Pose3 value) {
  values->insert(internal::PoseKey(i, t), value);
}

/// Insert pose for i-th link at time t.
void InsertPose(gtsam::Values *values, int i, gtsam::Pose3 value) {
  values->insert(internal::PoseKey(i), value);
}

/// Retrieve pose for i-th link at time t.
gtsam::Pose3 Pose(const gtsam::Values &values, int i, int t) {
  return values.at<gtsam::Pose3>(internal::PoseKey(i, t));
};

} // namespace gtdynamics
