/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  values.cpp
 * @brief Shortcots for inserting/retrieving variables in/from Values.
 * @author: Frank Dellaert
 * @author: Varun Agrawal
 */

#include <gtdynamics/utils/values.h>

namespace gtdynamics {

using gtsam::Pose3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;
using gtsam::VectorValues;

/* ************************************************************************* */
const char *KeyDoesNotExist::what() const noexcept {
  if (gtd_message_.empty())
    gtd_message_ = "Attempting to " + std::string(operation_) + " the key \"" +
                   _GTDKeyFormatter(key_) +
                   "\", which does not exist in the Values.";
  return gtd_message_.c_str();
}

/* ************************************************************************* */
void InsertJointAngle(Values *values, int j, int t, double value) {
  values->insert(internal::JointAngleKey(j, t), value);
}

void InsertJointAngle(Values *values, int j, double value) {
  values->insert(internal::JointAngleKey(j), value);
}

Vector JointAngle(const VectorValues &values, int j, int t) {
  return values.at(internal::JointAngleKey(j, t));
}

double JointAngle(const Values &values, int j, int t) {
  return internal::at<double>(values, internal::JointAngleKey(j, t));
}

/* ************************************************************************* */
void InsertJointVel(Values *values, int j, int t, double value) {
  values->insert(internal::JointVelKey(j, t), value);
}

void InsertJointVel(Values *values, int j, double value) {
  values->insert(internal::JointVelKey(j), value);
}

Vector JointVel(const VectorValues &values, int j, int t) {
  return values.at(internal::JointVelKey(j, t));
}

double JointVel(const Values &values, int j, int t) {
  return internal::at<double>(values, internal::JointVelKey(j, t));
}

/* ************************************************************************* */
void InsertJointAccel(Values *values, int j, int t, double value) {
  values->insert(internal::JointAccelKey(j, t), value);
}

void InsertJointAccel(Values *values, int j, double value) {
  values->insert(internal::JointAccelKey(j), value);
}

Vector JointAccel(const VectorValues &values, int j, int t) {
  return values.at(internal::JointAccelKey(j, t));
}

double JointAccel(const Values &values, int j, int t) {
  return internal::at<double>(values, internal::JointAccelKey(j, t));
}


/* ************************************************************************* */
void InsertTorque(Values *values, int j, int t, double value) {
  values->insert(internal::TorqueKey(j, t), value);
}

void InsertTorque(Values *values, int j, double value) {
  values->insert(internal::TorqueKey(j), value);
}

Vector Torque(const VectorValues &values, int j, int t) {
  return values.at(internal::TorqueKey(j, t));
};

double Torque(const Values &values, int j, int t) {
  return internal::at<double>(values, internal::TorqueKey(j, t));
};

/* ************************************************************************* */
/// Insert pose for i-th link at time t.
void InsertPose(Values *values, int i, int t, Pose3 value) {
  values->insert(internal::PoseKey(i, t), value);
}

/// Insert pose for i-th link at time t.
void InsertPose(Values *values, int i, Pose3 value) {
  values->insert(internal::PoseKey(i), value);
}

/// Retrieve pose for i-th link at time t.
Pose3 Pose(const Values &values, int i, int t) {
  return internal::at<Pose3>(values, internal::PoseKey(i, t));
};

/* ************************************************************************* */
void InsertTwist(Values *values, int j, int t, Vector6 value) {
  values->insert(internal::TwistKey(j, t), value);
}

void InsertTwist(Values *values, int j, Vector6 value) {
  values->insert(internal::TwistKey(j), value);
}

Vector Twist(const VectorValues &values, int j, int t) {
  return values.at(internal::TwistKey(j, t));
}

Vector6 Twist(const Values &values, int j, int t) {
  return internal::at<Vector6>(values, internal::TwistKey(j, t));
}

/* ************************************************************************* */
void InsertTwistAccel(Values *values, int j, int t, Vector6 value) {
  values->insert(internal::TwistAccelKey(j, t), value);
}

void InsertTwistAccel(Values *values, int j, Vector6 value) {
  values->insert(internal::TwistAccelKey(j), value);
}

Vector TwistAccel(const VectorValues &values, int j, int t) {
  return values.at(internal::TwistAccelKey(j, t));
}

Vector6 TwistAccel(const Values &values, int j, int t) {
  return internal::at<Vector6>(values, internal::TwistAccelKey(j, t));
}

/* ************************************************************************* */
void InsertWrench(Values *values, int i, int j, int t, Vector6 value) {
  values->insert(internal::WrenchKey(i, j, t), value);
}

void InsertWrench(Values *values, int i, int j, Vector6 value) {
  values->insert(internal::WrenchKey(i, j), value);
}

Vector Wrench(const VectorValues &values, int i, int j, int t) {
  return values.at(internal::WrenchKey(i, j, t));
}

Vector6 Wrench(const Values &values, int i, int j, int t) {
  return internal::at<Vector6>(values, internal::WrenchKey(i, j, t));
}

} // namespace gtdynamics
