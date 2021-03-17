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

namespace gtdynamics {

/**
 * JointTyped is a convenience class that inherits from Joint which wraps
 * abstract transformXXX that take in joint type arguments into transformXXX
 * from Joint which take in gtsam::Values object
 */

// TODO(Gerry) JointTyped was an intermediate step towards adding ball and
// sphere joints but we never finished it because for other joint types,
// transformFrom and transformTo can't just use a double as the joint angle
// argument, they need Unit3 or Rot3

class JointTyped : public Joint {
 public:
  using This = JointTyped;

  using JointCoordinateType = double;         // standin for template
  using JointCoordinateTangentType = double;  // standin for template
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

  /// Inherit overloaded functions
  using Joint::transformFrom;
  using Joint::transformTwistAccelFrom;
  using Joint::transformTwistFrom;

  /**
   * @name Abstract
   * Joint classes must implement these methods.
   */
  ///@{

  /**
   * Abstract method. Return the transform from the other link com to this link
   * com frame.
   */
  virtual Pose3 transformTo(
      const LinkSharedPtr &link, JointCoordinate q,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const = 0;

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
  virtual gtsam::Matrix6 AdjointMapJacobianJointAngle(
      const LinkSharedPtr &link, JointCoordinate q) const = 0;

  ///@}

  /**
   * @name transformConvenience
   * These are convenience functions to provide more argument options for the
   * transform functions.
   */
  ///@{

  /// Convenience method. Return the pose of this link com
  Pose3 transformTo(
      const LinkSharedPtr &link, JointCoordinate q, const gtsam::Pose3 &T_other,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, 6> H_T_other = boost::none) const {
    gtsam::Matrix66 H_relPose;
    Pose3 error = T_other.compose(transformFrom(link, q, H_q), H_T_other,
                                  H_q ? &H_relPose : 0);
    if (H_q) {
      *H_q = H_relPose * (*H_q);
    }
    return error;
  }

  /**
   * Convenience method to return the transform from this link com to the other
   * link com frame.
   */
  Pose3 transformFrom(const LinkSharedPtr &link, JointCoordinate q,
                      gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    return transformTo(otherLink(link), q, H_q);
  }

  /// Convenience method to return the pose of other link com.
  Pose3 transformFrom(
      const LinkSharedPtr &link, JointCoordinate q, const gtsam::Pose3 &T_this,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, 6> H_T_this = boost::none) const {
    return transformTo(otherLink(link), q, T_this, H_q, H_T_this);
  }

  /**
   * Convenience method to return the twist of the other link given this link's
   * twist and joint angle.
   */
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr &link, JointCoordinate q, JointVelocity q_dot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none) const {
    return transformTwistTo(otherLink(link), q, q_dot, this_twist, H_q, H_q_dot,
                            H_this_twist);
  }

  /** Convenience method. Return the twist acceleration of the other link given
   * this link's twist accel and a Values object containing this joint's angle
   * value and derivatives.
   */
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr &link, JointCoordinate q, JointVelocity q_dot,
      JointAcceleration q_ddot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Vector6> this_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist_accel = boost::none) const {
    return transformTwistAccelTo(otherLink(link), q, q_dot, q_ddot, other_twist,
                                 this_twist_accel, H_q, H_q_dot, H_q_ddot,
                                 H_other_twist, H_this_twist_accel);
  }

  /**
   * Return the transform from the other link com to this link com frame given a
   * Values object containing this joint's angle value.
   * @throw ValuesKeyDoesNotExist if the appropriate key is missing from values
   */
  Pose3 transformTo(
      const LinkSharedPtr &link, const gtsam::Values &q,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    return transformTo(link, q.at<JointCoordinate>(key()), H_q);
  }

  /**
   * Return the twist of this link given the other link's twist and a Values
   * object containing this joint's angle Value.
   * @throw ValuesKeyDoesNotExist if the appropriate key is missing from values
   */
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, const gtsam::Values &q_and_q_dot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist =
          boost::none) const override {
    return transformTwistTo(link, q_and_q_dot.at<JointCoordinate>(key()),
                            q_and_q_dot.at<JointVelocity>(key()), other_twist,
                            H_q, H_q_dot, H_other_twist);
  }

  /** Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle value and
   * derivatives.
   * @throw ValuesKeyDoesNotExist if the appropriate key is missing from values
   */
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, const gtsam::Values &q_and_q_dot_and_q_ddot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist_accel =
          boost::none) const override {
    return transformTwistAccelTo(
        link, q_and_q_dot_and_q_ddot.at<JointCoordinate>(key()),
        q_and_q_dot_and_q_ddot.at<JointVelocity>(key()),
        q_and_q_dot_and_q_ddot.at<JointAcceleration>(key()), this_twist,
        other_twist_accel, H_q, H_q_dot, H_q_ddot, H_this_twist,
        H_other_twist_accel);
  }

  ///@}

  /// Return joint pose factors.
  gtsam::NonlinearFactorGraph qFactors(
      size_t t, const OptimizerSetting &opt) const override;

  /// Return joint vel factors.
  gtsam::NonlinearFactorGraph vFactors(
      size_t t, const OptimizerSetting &opt) const override;

  /// Return joint accel factors.
  gtsam::NonlinearFactorGraph aFactors(
      size_t t, const OptimizerSetting &opt) const override;

  /// Return joint dynamics factors.
  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override;
};

}  // namespace gtdynamics
