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

/* ************************************************************************* */
gtsam::Vector JointAngle(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointAngleKey(j, t));
}

/* ************************************************************************* */
gtsam::Vector JointVel(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointVelKey(j, t));
}

/* ************************************************************************* */
gtsam::Vector JointAccel(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::JointAccelKey(j, t));
}

/* ************************************************************************* */
/// Retrieve torque on the j-th joint at time t.
gtsam::Vector Torque(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::TorqueKey(j, t));
};

/* ************************************************************************* */
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

/* ************************************************************************* */
void InsertTwist(gtsam::Values *values, int j, int t, gtsam::Vector6 value) {
  values->insert(internal::TwistKey(j, t), value);
}

void InsertTwist(gtsam::Values *values, int j, gtsam::Vector6 value) {
  values->insert(internal::TwistKey(j), value);
}

gtsam::Vector Twist(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::TwistKey(j, t));
}

gtsam::Vector6 Twist(const gtsam::Values &values, int j, int t) {
  return values.at<gtsam::Vector6>(internal::TwistKey(j, t));
}

/* ************************************************************************* */
void InsertTwistAccel(gtsam::Values *values, int j, int t, gtsam::Vector6 value) {
  values->insert(internal::TwistAccelKey(j, t), value);
}

void InsertTwistAccel(gtsam::Values *values, int j, gtsam::Vector6 value) {
  values->insert(internal::TwistAccelKey(j), value);
}

gtsam::Vector TwistAccel(const gtsam::VectorValues &values, int j, int t) {
  return values.at(internal::TwistAccelKey(j, t));
}

gtsam::Vector6 TwistAccel(const gtsam::Values &values, int j, int t) {
  return values.at<gtsam::Vector6>(internal::TwistAccelKey(j, t));
}

/* ************************************************************************* */
void InsertWrench(gtsam::Values *values, int i, int j, int t, gtsam::Vector6 value) {
  values->insert(internal::WrenchKey(i, j, t), value);
}

void InsertWrench(gtsam::Values *values, int i, int j, gtsam::Vector6 value) {
  values->insert(internal::WrenchKey(i, j), value);
}

gtsam::Vector Wrench(const gtsam::VectorValues &values, int i, int j, int t) {
  return values.at(internal::WrenchKey(i, j, t));
}

gtsam::Vector6 Wrench(const gtsam::Values &values, int i, int j, int t) {
  return values.at<gtsam::Vector6>(internal::WrenchKey(i, j, t));
}

} // namespace gtdynamics
