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

#define GTD_PRINT(x) ((x).print(#x, gtdynamics::_GTDKeyFormatter))

namespace gtdynamics {

/// Custom exception that properly formats dynamics keys.
class KeyDoesNotExist : public gtsam::ValuesKeyDoesNotExist {
  mutable std::string gtd_message_;

 public:
  using gtsam::ValuesKeyDoesNotExist::ValuesKeyDoesNotExist;
  ~KeyDoesNotExist() noexcept override {}

  /// The message to be displayed to the user
  const char *what() const noexcept override;
};

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

/// Custom retrieval that throws KeyDoesNotExist
template <typename T>
T at(const gtsam::Values &values, size_t key) {
  try {
    return values.at<T>(key);
  } catch (const gtsam::ValuesKeyDoesNotExist &e) {
    throw KeyDoesNotExist("at", e.key());
  }
}

}  // namespace internal

/* *************************************************************************
  Functions for Joint Angles.
 ************************************************************************* */

/**
 * @brief Insert j-th joint angle at time t.
 *
 * @tparam T Joint angle type (default `double`).
 * @param values Values pointer to insert joint angle into.
 * @param j The joint id.
 * @param t Time step.
 * @param value The joint angle value.
 */
template <typename T = double>
void InsertJointAngle(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointAngleKey(j, t), value);
}

/**
 * @brief Insert j-th joint angle at time 0.
 *
 * @tparam T Joint angle type (default `double`).
 * @param values Values pointer to insert joint angle into.
 * @param j The joint id.
 * @param value The joint angle value.
 */
template <typename T = double>
void InsertJointAngle(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointAngleKey(j), value);
}

/**
 * @brief Retrieve j-th joint angle at time t as a Vector.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the joint angle.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector JointAngle(const gtsam::VectorValues &values, int j, int t = 0);

/**
 * @brief Retrieve j-th joint angle at time t.
 *
 * @tparam T Joint angle type (default is `double`).
 * @param values Values dictionary containing the joint angle.
 * @param j The joint id.
 * @param t Time step.
 * @return T
 */
template <typename T = double>
T JointAngle(const gtsam::Values &values, int j, int t = 0) {
  return internal::at<T>(values, internal::JointAngleKey(j, t));
}

/* *************************************************************************
  Functions for Joint Velocities.
 ************************************************************************* */

/**
 * @brief Insert j-th joint velocity at time t.
 *
 * @tparam T Joint velocity type (default `double`).
 * @param values Values dictionary pointer to insert joint velocity into.
 * @param j The joint id.
 * @param t Time step.
 * @param value The joint velocity value.
 */
template <typename T = double>
void InsertJointVel(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointVelKey(j, t), value);
}

/**
 * @brief Insert j-th joint velocity at time 0.
 *
 * @tparam T Joint velocity type (default `double`).
 * @param values Values pointer to insert joint velocity into.
 * @param j The joint id.
 * @param value The joint velocity value.
 */
template <typename T = double>
void InsertJointVel(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointVelKey(j), value);
}

/**
 * @brief Retrieve j-th joint velocity at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the joint velocity.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector JointVel(const gtsam::VectorValues &values, int j, int t = 0);

/**
 * @brief Retrieve j-th joint velocity at time t.
 *
 * @tparam T Joint velocity type (default is `double`).
 * @param values Values dictionary containing the joint velocity.
 * @param j The joint id.
 * @param t Time step.
 * @return T
 */
template <typename T = double>
T JointVel(const gtsam::Values &values, int j, int t = 0) {
  return internal::at<T>(values, internal::JointVelKey(j, t));
}

/* *************************************************************************
  Functions for Joint Accelerations.
 ************************************************************************* */

/**
 * @brief Insert j-th joint acceleration at time t.
 *
 * @tparam T Joint acceleration type (default `double`).
 * @param values Values dictionary pointer to insert joint acceleration into.
 * @param j The joint id.
 * @param t Time step.
 * @param value The joint acceleration value.
 */
template <typename T = double>
void InsertJointAccel(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::JointAccelKey(j, t), value);
}

/**
 * @brief Insert j-th joint acceleration at time 0.
 *
 * @tparam T Joint acceleration type (default `double`).
 * @param values Values pointer to insert joint acceleration into.
 * @param j The joint id.
 * @param value The joint acceleration value.
 */
template <typename T = double>
void InsertJointAccel(gtsam::Values *values, int j, T value) {
  values->insert(internal::JointAccelKey(j), value);
}

/**
 * @brief Retrieve j-th joint acceleration at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the joint acceleration.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector JointAccel(const gtsam::VectorValues &values, int j, int t = 0);
 
/**
 * @brief Retrieve j-th joint acceleration at time t.
 *
 * @tparam T Joint acceleration type (default is `double`).
 * @param values Values dictionary containing the joint acceleration.
 * @param j The joint id.
 * @param t Time step.
 * @return T
 */
template <typename T = double>
T JointAccel(const gtsam::Values &values, int j, int t = 0) {
  return internal::at<T>(values, internal::JointAccelKey(j, t));
}

/* *************************************************************************
  Functions for Torques.
 ************************************************************************* */

/**
 * @brief Insert torque on the j-th joint at time t.
 *
 * @tparam T Torque type (default `double`).
 * @param values Values dictionary pointer to insert torque into.
 * @param j The joint id.
 * @param t Time step.
 * @param value The torque value.
 */
template <typename T = double>
void InsertTorque(gtsam::Values *values, int j, int t, T value) {
  values->insert(internal::TorqueKey(j, t), value);
}

/**
 * @brief Insert torque on the j-th joint at time 0.
 *
 * @tparam T Torque type (default `double`).
 * @param values Values pointer to insert torque into.
 * @param j The joint id.
 * @param value The torque value.
 */
template <typename T = double>
void InsertTorque(gtsam::Values *values, int j, T value) {
  values->insert(internal::TorqueKey(j), value);
}

/**
 * @brief Retrieve torque on the j-th joint at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the torque.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector Torque(const gtsam::VectorValues &values, int j, int t = 0);

/**
 * @brief Retrieve torque on the j-th joint at time t.
 *
 * @tparam T Joint torque type (default is `double`).
 * @param values Values dictionary containing the joint torque.
 * @param j The joint id.
 * @param t Time step.
 * @return T
 */
template <typename T = double>
T Torque(const gtsam::Values &values, int j, int t = 0) {
  return internal::at<T>(values, internal::TorqueKey(j, t));
};

/* *************************************************************************
  Functions for Poses.
 ************************************************************************* */

/**
 * @brief Insert pose for i-th link at time t.
 *
 * @param values Values dictionary pointer to insert Pose3 into.
 * @param i The link id.
 * @param t Time step.
 * @param value The Pose3 value.
 */
void InsertPose(gtsam::Values *values, int i, int t, gtsam::Pose3 value);

/**
 * @brief Insert pose for i-th link at time 0.

 * @param values Values pointer to insert Pose3 into.
 * @param i The link id.
 * @param value The Pose3 value.
 */
void InsertPose(gtsam::Values *values, int i, gtsam::Pose3 value);

/**
 * @brief Retrieve pose for i-th link at time t.
 *
 * @param values Values dictionary containing the Pose3.
 * @param i The link id.
 * @param t Time step.
 * @return gtsam::Pose3
 */
gtsam::Pose3 Pose(const gtsam::Values &values, int i, int t = 0);

/* *************************************************************************
  Functions for Twists.
 ************************************************************************* */

/**
 * @brief Insert j-th twist at time t.
 *
 * @param values Values dictionary pointer to insert twist into.
 * @param j The joint id.
 * @param t Time step.
 * @param value 6 dimensional twist vector.
 */
void InsertTwist(gtsam::Values *values, int j, int t, gtsam::Vector6 value);

/**
 * @brief Insert j-th twist at time 0.
 *
 * @param values Values pointer to insert twist into.
 * @param j The joint id.
 * @param value 6 dimensional twist vector.
 */
void InsertTwist(gtsam::Values *values, int j, gtsam::Vector6 value);

/**
 * @brief Retrieve j-th twist at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the twist.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector Twist(const gtsam::VectorValues &values, int j, int t = 0);

/**
 * @brief Retrieve j-th twist at time t.
 *
 * @param values Values dictionary containing the twist.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector6
 */
gtsam::Vector6 Twist(const gtsam::Values &values, int j, int t = 0);

/* *************************************************************************
  Functions for Twist Accelerations.
 ************************************************************************* */

/**
 * @brief Insert j-th twist acceleration at time t.
 *
 * @param values Values dictionary pointer to insert twist acceleration into.
 * @param j The joint id.
 * @param t Time step.
 * @param value 6 dimensional twist acceleration vector.
 */
void InsertTwistAccel(gtsam::Values *values, int j, int t,
                      gtsam::Vector6 value);

/**
 * @brief Insert j-th twist acceleration at time 0.
 *
 * @param values Values pointer to insert twist acceleration into.
 * @param j The joint id.
 * @param value 6 dimensional twist acceleration vector.
 */
void InsertTwistAccel(gtsam::Values *values, int j, gtsam::Vector6 value);

/**
 * @brief Retrieve j-th twist acceleration at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the twist acceleration.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector TwistAccel(const gtsam::VectorValues &values, int j, int t = 0);

/**
 * @brief Retrieve j-th twist acceleration at time t.
 *
 * @param values Values dictionary containing the twist acceleration.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector6
 */
gtsam::Vector6 TwistAccel(const gtsam::Values &values, int j, int t = 0);

/* *************************************************************************
  Functions for Wrenches.
 ************************************************************************* */

/**
 * @brief Insert wrench for i-th link and j-th joint at time t.
 *
 * @param values Values dictionary pointer to insert wrench into.
 * @param i The link id.
 * @param j The joint id.
 * @param t Time step.
 * @param value 6 dimensional wrench vector.
 */
void InsertWrench(gtsam::Values *values, int i, int j, int t,
                  gtsam::Vector6 value);

/**
 * @brief Insert wrench for i-th link and j-th joint at time 0.
 *
 * @param values Values pointer to insert wrench into.
 * @param i The link id.
 * @param j The joint id.
 * @param value 6 dimensional wrench vector.
 */
void InsertWrench(gtsam::Values *values, int i, int j, gtsam::Vector6 value);

/**
 * @brief Retrieve wrench for i-th link and j-th joint at time t.
 * Mainly used in the factor graph machinery.
 *
 * @param values Values dictionary containing the wrench.
 * @param i The link id.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector
 */
gtsam::Vector Wrench(const gtsam::VectorValues &values, int i, int j,
                     int t = 0);

/**
 * @brief Retrieve wrench for i-th link and j-th joint at time t.
 *
 * @param values Values dictionary containing the wrench.
 * @param i The link id.
 * @param j The joint id.
 * @param t Time step.
 * @return gtsam::Vector6
 */
gtsam::Vector6 Wrench(const gtsam::Values &values, int i, int j, int t = 0);

}  // namespace gtdynamics
