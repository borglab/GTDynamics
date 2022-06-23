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
  values->insert(JointAngleKey(j, t), value);
}

void InsertJointAngle(Values *values, int j, double value) {
  values->insert(JointAngleKey(j), value);
}

Vector JointAngle(const VectorValues &values, int j, int t) {
  return values.at(JointAngleKey(j, t));
}

double JointAngle(const Values &values, int j, int t) {
  return at<double>(values, JointAngleKey(j, t));
}

/* ************************************************************************* */
void InsertJointVel(Values *values, int j, int t, double value) {
  values->insert(JointVelKey(j, t), value);
}

void InsertJointVel(Values *values, int j, double value) {
  values->insert(JointVelKey(j), value);
}

Vector JointVel(const VectorValues &values, int j, int t) {
  return values.at(JointVelKey(j, t));
}

double JointVel(const Values &values, int j, int t) {
  return at<double>(values, JointVelKey(j, t));
}

/* ************************************************************************* */
void InsertJointAccel(Values *values, int j, int t, double value) {
  values->insert(JointAccelKey(j, t), value);
}

void InsertJointAccel(Values *values, int j, double value) {
  values->insert(JointAccelKey(j), value);
}

Vector JointAccel(const VectorValues &values, int j, int t) {
  return values.at(JointAccelKey(j, t));
}

double JointAccel(const Values &values, int j, int t) {
  return at<double>(values, JointAccelKey(j, t));
}

/* ************************************************************************* */
void InsertTorque(Values *values, int j, int t, double value) {
  values->insert(TorqueKey(j, t), value);
}

void InsertTorque(Values *values, int j, double value) {
  values->insert(TorqueKey(j), value);
}

Vector Torque(const VectorValues &values, int j, int t) {
  return values.at(TorqueKey(j, t));
};

double Torque(const Values &values, int j, int t) {
  return at<double>(values, TorqueKey(j, t));
};

/* ************************************************************************* */
/// Insert pose for i-th link at time t.
void InsertPose(Values *values, int i, int t, Pose3 value) {
  values->insert(PoseKey(i, t), value);
}

/// Insert pose for i-th link at time t.
void InsertPose(Values *values, int i, Pose3 value) {
  values->insert(PoseKey(i), value);
}

/// Retrieve pose for i-th link at time t.
Pose3 Pose(const Values &values, int i, int t) {
  return at<Pose3>(values, PoseKey(i, t));
};

/* ************************************************************************* */
void InsertTwist(Values *values, int j, int t, Vector6 value) {
  values->insert(TwistKey(j, t), value);
}

void InsertTwist(Values *values, int j, Vector6 value) {
  values->insert(TwistKey(j), value);
}

Vector Twist(const VectorValues &values, int j, int t) {
  return values.at(TwistKey(j, t));
}

Vector6 Twist(const Values &values, int j, int t) {
  return at<Vector6>(values, TwistKey(j, t));
}

/* ************************************************************************* */
void InsertTwistAccel(Values *values, int j, int t, Vector6 value) {
  values->insert(TwistAccelKey(j, t), value);
}

void InsertTwistAccel(Values *values, int j, Vector6 value) {
  values->insert(TwistAccelKey(j), value);
}

Vector TwistAccel(const VectorValues &values, int j, int t) {
  return values.at(TwistAccelKey(j, t));
}

Vector6 TwistAccel(const Values &values, int j, int t) {
  return at<Vector6>(values, TwistAccelKey(j, t));
}

/* ************************************************************************* */
void InsertWrench(Values *values, int i, int j, int t, Vector6 value) {
  values->insert(WrenchKey(i, j, t), value);
}

void InsertWrench(Values *values, int i, int j, Vector6 value) {
  values->insert(WrenchKey(i, j), value);
}

Vector Wrench(const VectorValues &values, int i, int j, int t) {
  return values.at(WrenchKey(i, j, t));
}

Vector6 Wrench(const Values &values, int i, int j, int t) {
  return at<Vector6>(values, WrenchKey(i, j, t));
}

}  // namespace gtdynamics
