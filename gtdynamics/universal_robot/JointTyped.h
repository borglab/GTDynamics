/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointTyped.h
 * @brief Specialized form of Joint for specific joint angle types
 * @author: Frank Dellaert
 * @author: Mandy Xie
 * @author: Alejandro Escontrela
 * @author: Yetong Zhang
 * @author: Varun Agrawal
 */

#pragma once

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * JointTyped is a convenience class that inherits from Joint which wraps
 * abstract transformXXX that take in joint type arguments into transformXXX
 * from Joint which take in gtsam::Values object
 */

// TODO(Gerry) JointTyped was an intermediate step towards adding ball and
// sphere joints but we never finished it because for other joint types,
// relativePoseOf can't just use a double as the joint angle
// argument, they need Unit3 or Rot3

class JointTyped : public Joint {
public:
  using This = JointTyped;

  using JointCoordinateType = double;        // standin for template
  using JointCoordinateTangentType = double; // standin for template
  using JointCoordinate = JointCoordinateType;
  using JointVelocity = JointCoordinateTangentType;
  using JointAcceleration = JointCoordinateTangentType;
  using JointTorque = JointCoordinateTangentType;

  enum { N = gtsam::traits<JointCoordinate>::dimension };
  using VectorN = Eigen::Matrix<double, N, 1>;
  using MatrixN = Eigen::Matrix<double, N, N>;

public:
  /// Inherit constructors
  using Joint::Joint;

  /**
   * @name Abstract
   * Joint classes must implement these methods.
   */
  ///@{

  /**
   * Abstract method. Return the pose of the child link in the parent link
   * frame, given the joint coordinate.
   */
  virtual Pose3
  parentTchild(JointCoordinate q,
               gtsam::OptionalJacobian<6, N> H_q = boost::none) const = 0;

  /**
   * Abstract method. Return the pose of the parent link in the child link
   * frame, given the joint coordinate.
   */
  virtual Pose3
  childTparent(JointCoordinate q,
               gtsam::OptionalJacobian<6, N> H_q = boost::none) const = 0;

  /// Joint-induced twist in child frame
  virtual gtsam::Vector6 childTwist(JointVelocity q_dot) const = 0;

  /// Joint-induced twist in parent frame
  virtual gtsam::Vector6 parentTwist(JointVelocity q_dot) const = 0;

  /**
   * Abstract method. Return the twist of this link given the other link's twist
   * and joint angle.
   */
  virtual gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, JointCoordinate q, JointVelocity q_dot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const = 0;

  /** Abstract method. Return the twist acceleration of this link given the
   * other link's twist acceleration, both links' twists, and joint angle.
   */
  virtual gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, JointCoordinate q, JointVelocity q_dot,
      JointAcceleration q_ddot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const = 0;

  /// Abstract method. Return the torque on this joint given the wrench
  virtual JointTorque transformWrenchToTorque(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<N, 6> H_wrench = boost::none) const = 0;

  /// Calculate AdjointMap jacobian w.r.t. joint coordinate q.
  /// TODO(gerry + stephanie): change to calculate the jacobian of Ad_T(v) wrt T
  /// rather than jacobian of Ad_T wrt q (and put in utils or PR to GTSAM)
  virtual gtsam::Matrix6
  AdjointMapJacobianJointAngle(const LinkSharedPtr &link,
                               JointCoordinate q) const = 0;

  ///@}

  /**
   * @name transformConvenience
   * These are convenience functions to provide more argument options for the
   * transform functions.
   */
  ///@{

  /**
   * Return the pose of the child link in the parent link frame, given a Values
   * object containing the joint coordinate.
   */
  Pose3 parentTchild(
      const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    return parentTchild(JointAngle<JointCoordinate>(q, id(), t), H_q);
  }

  /**
   * Return the pose of the parent link in the child link frame, given a Values
   * object containing the joint coordinate.
   */
  Pose3 childTparent(
      const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    return childTparent(JointAngle<JointCoordinate>(q, id(), t), H_q);
  }

  /// Joint-induced twist in child frame
  gtsam::Vector6 childTwist(const gtsam::Values &values,
                            size_t t = 0) const override {
    return childTwist(JointVel<JointVelocity>(values, id(), t));
  }

  /// Joint-induced twist in parent frame
  gtsam::Vector6 parentTwist(const gtsam::Values &values,
                             size_t t = 0) const override {
    return parentTwist(JointVel<JointVelocity>(values, id(), t));
  }

  /**
   * Return the relative pose of the specified link [link2] in the other link's
   * [link1] reference frame.
   */
  Pose3 relativePoseOf(const LinkSharedPtr &link2, JointCoordinate q,
                       gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    return isChildLink(link2) ? parentTchild(q, H_q) : childTparent(q, H_q);
  }

  /**
   * Return the relative pose of the specified link [link2] in
   * the other link's [link1] reference frame.
   * @throw KeyDoesNotExist if the appropriate key is missing from values
   */
  Pose3 relativePoseOf(
      const LinkSharedPtr &link2, const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    return relativePoseOf(link2, JointAngle<JointCoordinate>(q, id(), t), H_q);
  }

  /**
   * Return the world pose of the specified link [link2], given
   * the world pose of the other link [link1].
   */
  Pose3 poseOf(const LinkSharedPtr &link2, const Pose3 &wT1, JointCoordinate q,
               gtsam::OptionalJacobian<6, 6> H_wT1 = boost::none,
               gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    auto T12 = relativePoseOf(link2, q, H_q);
    return wT1.compose(T12, H_wT1); // H_wT2_T12 is identity
  }

  /**
   * Return the twist of this link given the other link's twist and a Values
   * object containing this joint's angle Value.
   * @param values containing q, q_dot
   * @throw ValuesKeyDoesNotExist if the appropriate key is missing from values
   */
  gtsam::Vector6 transformTwistTo(
      size_t t, const LinkSharedPtr &link, const gtsam::Values &values,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist = boost::none) //
      const override {
    return transformTwistTo(link, JointAngle<JointCoordinate>(values, id(), t),
                            JointVel<JointVelocity>(values, id(), t),
                            other_twist, H_q, H_q_dot, H_other_twist);
  }

  /** Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle value and
   * derivatives.
   * @param values containing q, q_dot, and q_ddot
   * @throw ValuesKeyDoesNotExist if the appropriate key is missing from values
   */
  gtsam::Vector6 transformTwistAccelTo(
      size_t t, const LinkSharedPtr &link, const gtsam::Values &values,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist_accel =
          boost::none) const override {
    return transformTwistAccelTo(link,
                                 JointAngle<JointCoordinate>(values, id(), t),
                                 JointVel<JointVelocity>(values, id(), t),
                                 JointAccel<JointAcceleration>(values, id(), t),
                                 this_twist, other_twist_accel, H_q, H_q_dot,
                                 H_q_ddot, H_this_twist, H_other_twist_accel);
  }

  ///@}

  /// Return joint pose factors.
  gtsam::NonlinearFactorGraph
  qFactors(size_t t, const OptimizerSetting &opt) const override;

  /// Return joint vel factors.
  gtsam::NonlinearFactorGraph
  vFactors(size_t t, const OptimizerSetting &opt) const override;

  /// Return joint accel factors.
  gtsam::NonlinearFactorGraph
  aFactors(size_t t, const OptimizerSetting &opt) const override;

  /// Return joint dynamics factors.
  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override;
};

} // namespace gtdynamics
