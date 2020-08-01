/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointTyped.h
 * @brief Specialized form of Joint for specific joint angle types
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#pragma once

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {

/**
 * JointTyped is a convenience class that inherits from Joint which wraps
 * transformXXXImpl that take in joint type argument into transformXXX
 * which take in gtsam::Values object
 */
class JointTyped : public Joint {
 public:
  typedef double JointAngleType;
  typedef double JointAngleTangentType;
  typedef JointAngleType AngleType;
  // TODO(gerry+stephanie): fix this for double
  // typedef typename AngleType::TangentVector JointAngleTangentType;
  typedef JointAngleTangentType AngleTangentType;
  enum { N = gtsam::traits<AngleType>::dimension };
  typedef Eigen::Matrix<double, N, 1> VectorN;
  typedef Eigen::Matrix<double, N, N> MatrixN;
  typedef JointTyped This;

 protected:
  /// Abstract method. Return the transform from the other link com to this link
  /// com frame
  virtual Pose3 transformToImpl(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const = 0;

  /// Abstract method. Return the twist of this link given the other link's
  /// twist and joint angle.
  virtual gtsam::Vector6 transformTwistToImpl(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q = boost::none,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const = 0;

  virtual gtsam::Vector6 transformTwistAccelToImpl(
      const LinkSharedPtr &link, boost::optional<AngleType> q = boost::none,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<AngleTangentType> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const = 0;

  virtual AngleTangentType transformWrenchToTorqueImpl(
      const LinkSharedPtr & link,
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<N, 6> H_wrench = boost::none) const = 0;

 public:
  /// Inherit constructors
  using Joint::Joint;

  /// Inherit overloaded functions
  using Joint::transformFrom;
  using Joint::transformTwistFrom;
  using Joint::transformTwistAccelFrom;

  /// Convenience method. Return the transform from this link com to the other
  /// link com frame
  Pose3 transformFrom(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    return transformToImpl(otherLink(link), q, H_q);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none) const {
    return transformTwistToImpl(otherLink(link), q, q_dot, this_twist, H_q,
                                H_q_dot, H_this_twist);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> this_twist,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none) const {
    return transformTwistToImpl(otherLink(link), boost::none, boost::none,
                                this_twist, H_q, H_q_dot, H_this_twist);
  }

  /// Convenience method. Return the twist acceleration of the other link given
  /// this link's twist accel and a Values object containing this joint's angle
  /// Value and derivatives.
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<AngleTangentType> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Vector6> this_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist_accel = boost::none) const {
    return transformTwistAccelToImpl(link, q, q_dot, q_ddot, other_twist,
                                     this_twist_accel, H_q, H_q_dot, H_q_ddot,
                                     H_other_twist, H_this_twist_accel);
  }

  /// Convenience method. Return the twist acceleration of the other link given
  /// this link's twist accel and a Values object containing this joint's angle
  /// Value and derivatives.
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> this_twist_accel,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist_accel = boost::none) const {
    return transformTwistAccelToImpl(otherLink(link),
                                     boost::none, boost::none, boost::none,
                                     boost::none, this_twist_accel,
                                     H_q, H_q_dot, H_q_ddot,
                                     H_other_twist, H_this_twist_accel);
  }

  /// Convenience method. Return the transform from this link com to the other
  /// link com frame
  Pose3 transformTo(
      const LinkSharedPtr &link, boost::optional<AngleType> q,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    return transformToImpl(link, q, H_q);
  }

  /// Return the transform from the other link com to this link
  /// com frame given a Values object containing this joint's angle Value
  Pose3 transformTo(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    if (q && q->exists<AngleType>(getKey()))
      return transformToImpl(link, q->at<AngleType>(getKey()), H_q);
    else
      return transformToImpl(link, boost::none, H_q);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const {
    return transformTwistToImpl(link, q, q_dot, other_twist, H_q, H_q_dot,
                                H_other_twist);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> other_twist,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const {
    return transformTwistToImpl(link, boost::none, boost::none, other_twist,
                                H_q, H_q_dot, H_other_twist);
  }

  /// Return the twist of this link given the other link's
  /// twist and a Values object containing this joint's angle Value.
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist =
          boost::none) const override {
    if (q && q->exists<AngleType>(getKey())) {
      if (q_dot && q_dot->exists<AngleTangentType>(getKey())) {
        return transformTwistToImpl(link,
            q->at<AngleType>(getKey()),
            q_dot->at<AngleTangentType>(getKey()),
            other_twist,
            H_q, H_q_dot, H_other_twist);
      } else {
        return transformTwistToImpl(link,
          q->at<AngleType>(getKey()),
          boost::none,
          other_twist,
          H_q, H_q_dot, H_other_twist);
      }
    } else {
      return transformTwistToImpl(link, boost::none, boost::none, other_twist,
                                  H_q, H_q_dot, H_other_twist);
    }
  }

  /// Convenience method. Return the twist acceleration of the other link given
  /// this link's twist acceleration and joint angle.
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link,
      boost::optional<AngleType> q,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<AngleTangentType> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel = boost::none) const {
    return transformTwistAccelToImpl(link, q, q_dot, q_ddot, this_twist,
                                     other_twist_accel, H_q, H_q_dot, H_q_ddot,
                                     H_this_twist, H_other_twist_accel);
  }

  /// Convenience method. Return the twist acceleration of the other link given
  /// this link's twist acceleration and joint angle.
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> other_twist_accel,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel = boost::none) const {
    return transformTwistAccelToImpl(link, boost::none, boost::none,
                                     boost::none, boost::none,
                                     other_twist_accel, H_q, H_q_dot, H_q_ddot,
                                     H_this_twist, H_other_twist_accel);
  }

  /// Return the twist acceleration of the other link given this link's
  /// twist accel and a Values object containing this joint's angle Value and
  /// derivatives.
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Values> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist_accel =
          boost::none) const override {
    if (q && q->exists<AngleType>(getKey())) {
      if (q_dot && q_dot->exists<AngleTangentType>(getKey())) {
        if (q_ddot && q_ddot->exists<AngleTangentType>(getKey())) {
          return transformTwistAccelToImpl(link,
              q->at<AngleType>(getKey()),
              q_dot->at<AngleTangentType>(getKey()),
              q_ddot->at<AngleTangentType>(getKey()),
              this_twist,
              other_twist_accel,
              H_q, H_q_dot, H_q_ddot, H_this_twist, H_other_twist_accel);
        } else {
          return transformTwistAccelToImpl(link,
              q->at<AngleType>(getKey()),
              q_dot->at<AngleTangentType>(getKey()),
              boost::none,
              this_twist,
              other_twist_accel,
              H_q, H_q_dot, H_q_ddot, H_this_twist, H_other_twist_accel);
        }
      } else {
        return transformTwistAccelToImpl(link,
          q->at<AngleType>(getKey()),
          boost::none,
          boost::none,
          this_twist,
          other_twist_accel,
          H_q, H_q_dot, H_q_ddot, H_this_twist, H_other_twist_accel);
      }
    } else {
      return transformTwistAccelToImpl(link, boost::none, boost::none,
                                       boost::none, this_twist,
                                       other_twist_accel,
                                       H_q, H_q_dot, H_q_ddot,
                                       H_this_twist, H_other_twist_accel);
    }
  }

  /// Return the torque of the joint given the wrench on the child link.
  JointAngleTangentType transformWrenchToTorque(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none) const {
    return transformWrenchToTorqueImpl(link, wrench, H_wrench);
  }

  /// Calculate AdjointMap jacobian w.r.t. joint coordinate q.
  /// TODO(gerry + stephanie): change to calculate the jacobian of Ad_T(v) wrt T
  /// rather than jacobian of Ad_T wrt q (and put in utils or PR to GTSAM)
  virtual gtsam::Matrix6 AdjointMapJacobianJointAngle(const LinkSharedPtr &link,
      boost::optional<AngleType> q = boost::none) const = 0;

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

}
