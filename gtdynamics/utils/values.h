/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  values.h
 * @brief Shortcots for inserting/retrieving variables in/from gtsam::Values.
 * @author: Frank Dellaert
 * @author: Varun Agrawal
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {

/* *************************************************************************
  Key definitions.
 ************************************************************************* */
namespace internal {
/// Shorthand for q_j_t, for j-th joint angle at time t.
inline DynamicsSymbol JointAngleKey(int j, int t = 0) {
  return DynamicsSymbol::JointSymbol("q", j, t);
}

/// Shorthand for v_j_t, for j-th joint velocity at time t.
inline DynamicsSymbol JointVelKey(int j, int t = 0) {
  return DynamicsSymbol::JointSymbol("v", j, t);
}

/// Shorthand for a_j_t, for j-th joint acceleration at time t.
inline DynamicsSymbol JointAccelKey(int j, int t = 0) {
  return DynamicsSymbol::JointSymbol("a", j, t);
}

/// Shorthand for T_j_t, for torque on the j-th joint at time t.
inline DynamicsSymbol TorqueKey(int j, int t = 0) {
  return DynamicsSymbol::JointSymbol("T", j, t);
}

/// Shorthand for p_i_t, for COM pose on the i-th link at time t.
inline DynamicsSymbol PoseKey(int i, int t = 0) {
  return DynamicsSymbol::LinkSymbol("p", i, t);
}

/// Shorthand for V_i_t, for 6D link twist vector on the i-th link.
inline DynamicsSymbol TwistKey(int i, int t = 0) {
  return DynamicsSymbol::LinkSymbol("V", i, t);
}

/// Shorthand for A_i_t, for twist accelerations on the i-th link at time t.
inline DynamicsSymbol TwistAccelKey(int i, int t = 0) {
  return DynamicsSymbol::LinkSymbol("A", i, t);
}

/// Shorthand for F_i_j_t, wrenches at j-th joint on the i-th link at time t.
inline DynamicsSymbol WrenchKey(int i, int j, int t = 0) {
  return DynamicsSymbol::LinkJointSymbol("F", i, j, t);
}

}  // namespace internal

/* *************************************************************************
  Functions for Joint Angles.
 ************************************************************************* */

/// Insert j-th joint angle at time t.
template <typename T = double>
void InsertJointAngle(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointAngleKey(j, t), value);
}

/// Insert j-th joint angle at time 0.
template <typename T = double>
void InsertJointAngle(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointAngleKey(j), value);
}

/// Retrieve j-th joint angle at time t.
gtsam::Vector JointAngle(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve j-th joint angle at time t.
template <typename T = double>
T JointAngle(const gtsam::Values &values, int j, int t = 0) {
  return values.at<T>(internal::JointAngleKey(j, t));
}

/* *************************************************************************
  Functions for Joint Velocities.
 ************************************************************************* */

/// Insert j-th joint velocity at time t.
template <typename T = double>
void InsertJointVel(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointVelKey(j, t), value);
}

/// Insert j-th joint velocity at time 0.
template <typename T = double>
void InsertJointVel(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointVelKey(j), value);
}

/// Retrieve j-th joint velocity at time t.
gtsam::Vector JointVel(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve j-th joint velocity at time t.
template <typename T = double>
T JointVel(const gtsam::Values &values, int j, int t = 0) {
  return values.at<T>(internal::JointVelKey(j, t));
}

/* *************************************************************************
  Functions for Joint Accelerations.
 ************************************************************************* */

/// Insert j-th joint acceleration at time t.
template <typename T = double>
void InsertJointAccel(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointAccelKey(j, t), value);
}

/// Insert j-th joint acceleration at time 0.
template <typename T = double>
void InsertJointAccel(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointAccelKey(j), value);
}

/// Retrieve j-th joint acceleration at time t.
gtsam::Vector JointAccel(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve j-th joint acceleration at time t.
template <typename T = double>
T JointAccel(const gtsam::Values &values, int j, int t = 0) {
  return values.at<T>(internal::JointAccelKey(j, t));
}

/* *************************************************************************
  Functions for Torques.
 ************************************************************************* */

/// Insert torque on the j-th joint at time t.
template <typename T = double>
void InsertTorque(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::TorqueKey(j, t), value);
}

/// Insert torque on the j-th joint at time 0.
template <typename T = double>
void InsertTorque(gtsam::Values *values, int j, T value) {
  values->insert(internal::TorqueKey(j), value);
}

/// Retrieve torque on the j-th joint at time t.
gtsam::Vector Torque(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve torque on the j-th joint at time t.
template <typename T = double>
T Torque(const gtsam::Values &values, int j, int t = 0) {
  return values.at<T>(internal::TorqueKey(j, t));
};

/* *************************************************************************
  Functions for Poses.
 ************************************************************************* */

/// Insert pose for i-th link at time t.
void InsertPose(gtsam::Values *values, int i, int t, gtsam::Pose3 value);

/// Insert pose for i-th link at time 0.
void InsertPose(gtsam::Values *values, int i, gtsam::Pose3 value);

/// Retrieve pose for i-th link at time t.
gtsam::Pose3 Pose(const gtsam::Values &values, int i, int t = 0);

/* *************************************************************************
  Functions for Twists.
 ************************************************************************* */

/// Insert j-th twist at time t.
void InsertTwist(gtsam::Values *values, int j, int t, gtsam::Vector6 value);

/// Insert j-th twist at time 0.
void InsertTwist(gtsam::Values *values, int j, gtsam::Vector6 value);

/// Retrieve j-th twist at time t.
gtsam::Vector Twist(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve j-th twist at time t.
gtsam::Vector6 Twist(const gtsam::Values &values, int j, int t = 0);

/* *************************************************************************
  Functions for Twist Accelerations.
 ************************************************************************* */

/// Insert j-th twist acceleration at time t.
void InsertTwistAccel(gtsam::Values *values, int j, int t, gtsam::Vector6 value);

/// Insert j-th twist acceleration at time 0.
void InsertTwistAccel(gtsam::Values *values, int j, gtsam::Vector6 value);

/// Retrieve j-th twist acceleration at time t.
gtsam::Vector TwistAccel(const gtsam::VectorValues &values, int j, int t = 0);

/// Retrieve j-th twist acceleration at time t.
gtsam::Vector6 TwistAccel(const gtsam::Values &values, int j, int t = 0);

/* *************************************************************************
  Functions for Wrenches.
 ************************************************************************* */

/// Insert wrench for i-th link and j-th joint at time t.
void InsertWrench(gtsam::Values *values, int i, int j, int t, gtsam::Vector6 value);

/// Insert wrench for i-th link and j-th joint at time 0.
void InsertWrench(gtsam::Values *values, int i, int j, gtsam::Vector6 value);

/// Retrieve wrench for i-th link and j-th joint at time t.
gtsam::Vector Wrench(const gtsam::VectorValues &values, int i, int j, int t = 0);

/// Retrieve wrench for i-th link and j-th joint at time t.
gtsam::Vector6 Wrench(const gtsam::Values &values, int i, int j, int t = 0);


}  // namespace gtdynamics
