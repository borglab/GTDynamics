/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJointBase.h
 * @brief Representation of screw-type robot joints. Revolute, Prismatic, and
 *  Screw subclasses
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 * @author Varun Agrawal
 */

#pragma once

#include <cmath>
#include <map>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose3.h>

#include "gtdynamics/factors/JointLimitFactor.h"
#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {
/**
 * @class ScrewJointBase is an implementation of the abstract Joint class
 *  which represents a screw-type joint and contains all necessary factor
 *  construction methods.
 *  It is the base class for RevoluteJoint, PrismaticJoint, and ScrewJoint.
 */
class ScrewJointBase : public Joint {
  using Pose3 = gtsam::Pose3;
  using Vector6 = gtsam::Vector6;
  using This = ScrewJointBase;

 public:
  using JointCoordinate = double;
  using JointVelocity = double;
  using JointAcceleration = double;
  using JointTorque = double;
  enum { N = gtsam::traits<JointCoordinate>::dimension };

  using SharedPtr = boost::shared_ptr<This>;
  using ConstSharedPtr = boost::shared_ptr<const This>;

 protected:
  gtsam::Vector3 axis_;

  // Screw axis in parent and child COM frames.
  Vector6 pScrewAxis_;
  Vector6 cScrewAxis_;

 public:
  /// Return transform of child link com frame w.r.t parent link com frame
  Pose3 parentTchild(double q,
                     gtsam::OptionalJacobian<6, 1> pMc_H_q = boost::none) const;

 protected:
  /// Return transform of parent link com frame w.r.t child link com frame
  Pose3 childTparent(double q,
                     gtsam::OptionalJacobian<6, 1> cMp_H_q = boost::none) const;

  /**
   * Return the joint axis in the joint frame. Rotational axis for revolute and
   * translation direction for prismatic in the joint frame.
   */
  const gtsam::Vector3 &axis() const { return axis_; }

 public:
  /// Inherit constructors
  using Joint::Joint;

  /**
   * Constructor using JointParams, joint name, wTj, screw axes,
   * and parent and child links.
   */
  ScrewJointBase(unsigned char id, const std::string &name, const Pose3 &wTj,
                 const LinkSharedPtr &parent_link,
                 const LinkSharedPtr &child_link, const JointParams &parameters,
                 const gtsam::Vector3 &axis, const Vector6 &jScrewAxis)
      : Joint(id, name, wTj, parent_link, child_link, parameters),
        axis_(axis),
        pScrewAxis_(-jTpcom_.inverse().AdjointMap() * jScrewAxis),
        cScrewAxis_(jTccom_.inverse().AdjointMap() * jScrewAxis) {}

  /// Return joint type for use in reconstructing robot from JointParams.
  Type type() const override { return Type::ScrewAxis; }

  /// Return screw axis expressed in the specified link frame
  const Vector6 screwAxis(const LinkSharedPtr &link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }


  /// Return a shared ptr to this joint.
  SharedPtr shared() {
    return boost::static_pointer_cast<This>(Joint::shared_from_this());
  }

  /// Return a const shared ptr to this joint.
  ConstSharedPtr shared() const {
    return boost::static_pointer_cast<const This>(Joint::shared_from_this());
  }

  // inherit overloads
  using Joint::poseOf;
  using Joint::relativePoseOf;
  using Joint::transformTwistAccelTo;
  using Joint::transformTwistTo;

  /**
   * Return the twist of this link given the other link's twist and joint angle.
   */
  Vector6 transformTwistTo(
      const LinkSharedPtr &link, double q, double q_dot,
      boost::optional<Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const;

  /**
   * Return the twist acceleration of this link given the other link's twist
   * acceleration, twist, and joint angle and this link's twist.
   */
  Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, double q, double q_dot, double q_ddot,
      boost::optional<Vector6> this_twist = boost::none,
      boost::optional<Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const;

  JointTorque transformWrenchToTorque(
      const LinkSharedPtr &link, boost::optional<Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<1, 6> H_wrench = boost::none) const;

  // TODO(frank): document and possibly eliminate
  gtsam::Matrix6 AdjointMapJacobianJointAngle(const LinkSharedPtr &link,
                                              double q) const {
    return AdjointMapJacobianQ(q, relativePoseOf(otherLink(link), q),
                               screwAxis(link));
  }

  /// Return forward dynamics priors on torque.
  gtsam::GaussianFactorGraph linearFDPriors(
      size_t t, const gtsam::Values &known_values,
      const OptimizerSetting &opt) const override;

  /// Return linearized acceleration factors.
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override;

  /// Return linearized dynamics factors.
  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override;


  /// Return joint limit factors.
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const override;

  /// Joint-induced twist in child frame
  gtsam::Vector6 childTwist(double q_dot) const {
    return cScrewAxis_ * q_dot;
  }

  /// Joint-induced twist in parent frame
  gtsam::Vector6 parentTwist(double q_dot) const {
    return pScrewAxis_ * q_dot;
  }

  /**
   * @name factors
   * Methods that create factors based on joint relationships.
   */
  ///@{

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

  ///@}

  /**
   * @name generic
   * These are methods that can be implemented here in terms of other methods.
   */
  ///@{

  /**
   * Return the relative pose of the specified link [link2] in the other link's
   * [link1] reference frame.
   */
  Pose3 relativePoseOf(const LinkSharedPtr &link2, JointCoordinate q,
                       gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    return isChildLink(link2) ? parentTchild(q, H_q) : childTparent(q, H_q);
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

  ///@}

  /**
   * @name valueBased
   * Methods that extract coordinates/velocities from Values and pass on to
   * abstract methods above.
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

};

}  // namespace gtdynamics
