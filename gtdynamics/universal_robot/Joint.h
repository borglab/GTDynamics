/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.h
 * @brief Abstract representation of a robot joint.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
// #include "gtdynamics/utils/DynamicsSymbol.h"

namespace gtdynamics {

// TODO(aescontrela): Make toString method to display joint info.

/* Shorthand for q_j_t, for j-th joint angle at time t. */
inline DynamicsSymbol JointAngleKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("q", j, t);
}

/* Shorthand for v_j_t, for j-th joint velocity at time t. */
inline DynamicsSymbol JointVelKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("v", j, t);
}

/* Shorthand for a_j_t, for j-th joint acceleration at time t. */
inline DynamicsSymbol JointAccelKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("a", j, t);
}

/* Shorthand for T_j_t, for torque on the j-th joint at time t. */
inline DynamicsSymbol TorqueKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("T", j, t);
}

/**
 * @class Joint is the base class for a joint connecting two Link objects.
 */
class Joint : public std::enable_shared_from_this<Joint> {
 public:
  /** joint effort types
   * Actuated: motor powered
   * Unactuated: not powered, free to move, exert zero torque
   * Impedance: with spring resistance
   */
  enum JointEffortType { Actuated, Unactuated, Impedance };

  enum JointType : char { Revolute = 'R', Prismatic = 'P', Screw = 'C' };

 protected:
  // This joint's name.
  std::string name_;

  // ID reference to DynamicsSymbol.
  int id_ = -1;

  LinkSharedPtr parent_link_;
  LinkSharedPtr child_link_;

  // Joint frame defined in world frame.
  gtsam::Pose3 wTj_;
  // Rest transform to parent link CoM frame from joint frame.
  gtsam::Pose3 jTpcom_;
  // Rest transform to child link CoM frame from joint frame.
  gtsam::Pose3 jTccom_;
  // Rest transform to parent link com frame from child link com frame at rest.
  gtsam::Pose3 pMccom_;

  /// Transform from the world frame to the joint frame.
  const gtsam::Pose3 &wTj() const { return wTj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const gtsam::Pose3 &jTpcom() const { return jTpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const gtsam::Pose3 &jTccom() const { return jTccom_; }

  /// Abstract method. Return transform of child link com frame w.r.t parent
  /// link com frame
  gtsam::Pose3 pMcCom(boost::optional<double> q = boost::none);

  /// Abstract method. Return transform of parent link com frame w.r.t child
  /// link com frame
  gtsam::Pose3 cMpCom(boost::optional<double> q = boost::none);

  /// Check if the link is a child link, throw an error if link is not
  /// connected to this joint.
  bool isChildLink(const LinkSharedPtr& link) const {
    LinkSharedPtr link_ptr = link;
    if (link_ptr != child_link_ && link_ptr != parent_link_)
      throw std::runtime_error("link " + link_ptr->name() +
                               " is not connected to this joint " + name_);
    return link_ptr == child_link_;
  }

 public:
  Joint() {}

  /**
   * @brief Constructor to create Joint from joint name, joint pose in
   * world frame, and shared pointers to the parent and child links.
   *
   * @param[in] name         name of joint
   * @param[in] wTj          joint pose expressed in world frame
   * @param[in] parent_link  Shared pointer to the parent Link.
   * @param[in] child_link   Shared pointer to the child Link.
   */
  Joint(const std::string &name, const gtsam::Pose3 &wTj,
        const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link)
      : name_(name),
        parent_link_(parent_link),
        child_link_(child_link),
        wTj_(wTj) {
    jTpcom_ = wTj_.inverse() * parent_link_->wTcom();
    jTccom_ = wTj_.inverse() * child_link_->wTcom();
    pMccom_ = parent_link_->wTcom().inverse() * child_link_->wTcom();
  }

  /**
   * @brief Default destructor.
   */
  virtual ~Joint() = default;

  /// Return a shared ptr to this joint.
  JointSharedPtr getSharedPtr() { return shared_from_this(); }

  /// Return a const shared ptr to this joint.
  JointConstSharedPtr getConstSharedPtr() const { return shared_from_this(); }

  /// Set the joint's ID to track reference to gtsam::LabeledSymbol.
  void setID(unsigned char id) { id_ = id; }

  /// Get the joint's ID.
  int getID() const {
    if (id_ == -1)
      throw std::runtime_error(
          "Calling getID on a link whose ID has not been set");
    return id_;
  }

  /// Return joint name.
  std::string name() const { return name_; }

  /// Return the connected link other than the one provided.
  LinkSharedPtr otherLink(const LinkSharedPtr& link) const {
    return isChildLink(link) ? parent_link_ : child_link_;
  }

  /// Return the links connected to this joint.
  std::vector<LinkSharedPtr> links() const {
    return std::vector<LinkSharedPtr>{parent_link_, child_link_};
  }

  /// Return a shared ptr to the parent link.
  LinkSharedPtr parentLink() const { return parent_link_; }

  /// Return a shared ptr to the child link.
  LinkSharedPtr childLink() const { return child_link_; }

  /**
   * \defgroup AbstractMethods Abstract methods for the joint class.
   * @{
   */

  /// Abstract method: Return joint type for use in reconstructing robot from
  /// Parameters.
  virtual JointType jointType() const = 0;

  /// Abstract method. Return the transform from the other link com to this link
  /// com frame
  virtual gtsam::Pose3 transformTo(
      const LinkSharedPtr link,
      boost::optional<const gtsam::Values&> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const = 0;

  /// Abstract method. Return the twist of the other link given this link's
  /// twist and a Values object containing this joint's angle Value.
  virtual gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist = boost::none) const = 0;

  /// Return the twist acceleration of the other link given this link's
  /// twist accel and a Values object containing this joint's angle Value and
  /// derivatives.
  virtual gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Values> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist_accel = boost::none) const = 0;

  /// Return the torque of the joint given the wrench on the child link.
  virtual gtsam::Values transformWrenchToTorque(
      boost::optional<gtsam::Vector6> wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none) const = 0;

  /** @fn (ABSTRACT) Return pose factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate q factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return pose factors.
   */
  virtual gtsam::NonlinearFactorGraph qFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /** @fn (ABSTRACT) Return velocity factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate v factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return velocity factors.
   */
  virtual gtsam::NonlinearFactorGraph vFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /** @fn (ABSTRACT) Return accel factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate a factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return accel factors.
   */
  virtual gtsam::NonlinearFactorGraph aFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /** @fn (ABSTRACT) Return linear accel factors in the dynamics graph.
   *
   * @param[in] t             The timestep for which to generate factors.
   * @param[in] poses         Link poses.
   * @param[in] twists        Link twists.
   * @param[in] joint_angles  Joint angles.
   * @param[in] joint_vels    Joint velocities.
   * @param[in] opt           OptimizerSetting object containing NoiseModels
   *    for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear accel factors.
   */
  virtual gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis =
          boost::none) const = 0;

  /** @fn (ABSTRACT) Return dynamics factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate dynamics factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return dynamics factors.
   */
  virtual gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const = 0;

  /** @fn (ABSTRACT) Return linear dynamics factors in the dynamics graph.
   *
   * @param[in] t             The timestep for which to generate factors.
   * @param[in] poses         Link poses.
   * @param[in] twists        Link twists.
   * @param[in] joint_angles  Joint angles.
   * @param[in] joint_vels    Joint velocities.
   * @param[in] opt           OptimizerSetting object containing NoiseModels
   *    for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear dynamics factors.
   */
  virtual gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis =
          boost::none) const = 0;

  /** @fn (ABSTRACT) Return joint limit factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate joint limit factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return joint limit factors.
   */
  virtual gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) = 0;

  /**@}*/

  /// Return the transform from this link com to the other link
  /// com frame given a Values object containing this joint's angle Value
  gtsam::Pose3 transformFrom(
      const LinkSharedPtr link,
      boost::optional<const gtsam::Values&> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const {
    return transformTo(otherLink(link), q, H_q);
  }

  /// Return the twist of the other link given this link's
  /// twist and a Values object containing this joint's angle Value.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none) const {
    return transformTwistTo(otherLink(link), q, q_dot, this_twist, H_q, H_q_dot,
                            H_this_twist);
  }

  /// Return the twist acceleration of the other link given this link's
  /// twist accel and a Values object containing this joint's angle Value and
  /// derivatives.
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Values> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Vector6> this_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist_accel = boost::none) const {
    return transformTwistAccelTo(otherLink(link), q, q_dot, q_ddot, other_twist,
                                 this_twist_accel, H_q, H_q_dot, H_q_ddot,
                                 H_other_twist, H_this_twist_accel);
  }
};

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

 protected:
  /// Abstract method. Return the transform from the other link com to this link
  /// com frame
  virtual gtsam::Pose3 transformToImpl(
      const LinkSharedPtr link,
      boost::optional<AngleType> q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const = 0;

  /// Abstract method. Return the twist of this link given the other link's
  /// twist and joint angle.
  virtual gtsam::Vector6 transformTwistToImpl(
      const LinkSharedPtr link,
      boost::optional<AngleType> q = boost::none,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const = 0;
  
  virtual gtsam::Vector6 transformTwistAccelToImpl(
      const LinkSharedPtr link,
      boost::optional<AngleType> q = boost::none,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<AngleTangentType> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel = boost::none) const = 0;

  virtual AngleTangentType transformWrenchToTorqueImpl(
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<6, N> H_wrench = boost::none) const = 0;

 public:
  /// Inherit constructors
  using Joint::Joint;

  /// Inherit overloaded functions
  using Joint::transformFrom;
  using Joint::transformTwistFrom;
  using Joint::transformTwistAccelFrom;

  /// Convenience method. Return the transform from this link com to the other link
  /// com frame
  gtsam::Pose3 transformFrom(
      const LinkSharedPtr link,
      boost::optional<AngleType> q,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    return transformToImpl(otherLink(link), q, H_q);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr link,
      boost::optional<AngleType> q,
      boost::optional<AngleTangentType> q_dot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none) const {
    return transformTwistToImpl(otherLink(link), q, q_dot, this_twist, H_q, H_q_dot,
                                  H_this_twist);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr link,
      boost::optional<gtsam::Vector6> this_twist,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none) const {
    return transformTwistToImpl(otherLink(link), boost::none, boost::none, this_twist,
                                  H_q, H_q_dot, H_this_twist);
  }

  /// Convenience method. Return the twist acceleration of the other link given
  /// this link's twist accel and a Values object containing this joint's angle
  /// Value and derivatives.
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr link,
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
      const LinkSharedPtr link,
      boost::optional<gtsam::Vector6> this_twist_accel,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist_accel = boost::none) const {
    return transformTwistFromImpl(link, boost::none, boost::none, boost::none, boost::none,
                                  this_twist_accel,
                                  H_q, H_q_dot, H_q_ddot, H_other_twist, H_this_twist_accel);
  }

  /// Convenience method. Return the transform from this link com to the other link
  /// com frame
  gtsam::Pose3 transformTo(
      const LinkSharedPtr link, boost::optional<AngleType> q,
      gtsam::OptionalJacobian<6, N> H_q = boost::none) const {
    return transformToImpl(link, q, H_q);
  }

  /// Return the transform from the other link com to this link
  /// com frame given a Values object containing this joint's angle Value
  gtsam::Pose3 transformTo(
      const LinkSharedPtr link,
      boost::optional<const gtsam::Values&> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    if (q && q->exists<AngleType>(getKey()))
      return transformToImpl(link, q->at<AngleType>(getKey()), H_q);
    else
      return transformToImpl(link, boost::none, H_q);
  }

  /// Convenience method. Return the twist of the other link given this link's
  /// twist and joint angle.
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr link,
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
      const LinkSharedPtr link,
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
      const LinkSharedPtr link,
      boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist = boost::none) const override {
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
      const LinkSharedPtr link,
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
      const LinkSharedPtr link,
      boost::optional<gtsam::Vector6> other_twist_accel,
      gtsam::OptionalJacobian<6, N> H_q = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, N> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel = boost::none) const {
    return transformTwistToAccelImpl(link, boost::none, boost::none,
                                     boost::none, boost::none,
                                     other_twist_accel, H_q, H_q_dot, H_q_ddot,
                                     H_this_twist, H_other_twist_accel);
  }

  /// Return the twist acceleration of the other link given this link's
  /// twist accel and a Values object containing this joint's angle Value and
  /// derivatives.
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr link, boost::optional<gtsam::Values> q = boost::none,
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
  gtsam::Values transformWrenchToTorque(
      boost::optional<gtsam::Vector6> wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none) const override {
    gtsam::Values torque;
    // torque.insert(key, transformWrenchToTorqueImpl(wrench, H_wrench));
    return torque;
  }
};

struct JointParams {
  std::string name;  // Name of this joint as described in the URDF file.

  JointEffortType jointEffortType = JointEffortType::Actuated;
  double springCoefficient = 0;      // spring coefficient for Impedance joint.
  double jointLimitThreshold = 0.0;  // joint angle limit threshold.
  double velocityLimitThreshold = 0.0;  // joint velocity limit threshold.
  double accelerationLimit = 10000;     // joint acceleration limit.
  double accelerationLimitThreshold = 0.0; // joint acceleration limit threshold.
  double torqueLimitThreshold = 0.0;  // joint torque limit threshold.
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_JOINT_H_
