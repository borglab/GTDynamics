/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.h
 * @brief Absract representation of a robot joint.
 * @author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

namespace gtdynamics {

/// Shorthand for q_j_t, for j-th joint angle at time t.
inline DynamicsSymbol JointAngleKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("q", j, t);
}

/// Shorthand for v_j_t, for j-th joint velocity at time t.
inline DynamicsSymbol JointVelKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("v", j, t);
}

/// Shorthand for a_j_t, for j-th joint acceleration at time t.
inline DynamicsSymbol JointAccelKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("a", j, t);
}

/// Shorthand for T_j_t, for torque on the j-th joint at time t.
inline DynamicsSymbol TorqueKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("T", j, t);
}

// TODO(G+S): change torque type from map<string, double> to gtsam::Values
/// Map from joint name to joint angle/vel/accel/torque
using JointValues = std::map<std::string, double>;

/// Joint is the base class for a joint connecting two Link objects.
class Joint : public boost::enable_shared_from_this<Joint> {
 protected:
  using Pose3 = gtsam::Pose3;

 public:
  /**
   * Joint effort types
   *
   * Actuated: motor powered
   * Unactuated: not powered, free to move, exert zero torque
   * Impedance: with spring resistance
   */
  enum EffortType { Actuated, Unactuated, Impedance };

  enum Type : char {
    Revolute = 'R',
    Prismatic = 'P',
    Screw = 'C',
    ScrewAxis = 'A'
  };

  /**
   * This struct contains information for scalar limits.
   * The lower and upper limits denote physical axis limits of the joint,
   * and the threshold is an error threshold used in calculations.
   */
  struct ScalarLimit {
    double value_lower_limit = -M_PI_2;
    double value_upper_limit = M_PI_2;
    double value_limit_threshold = 1e-9;
  };

  /**
   * This struct contains all parameters needed to construct a joint.
   */
  struct Parameters {
    EffortType effort_type = EffortType::Actuated;
    ScalarLimit scalar_limits;

    double velocity_limit = 10000.0;
    double velocity_limit_threshold = 0.0;
    double acceleration_limit = 10000.0;
    double acceleration_limit_threshold = 0.0;
    double torque_limit = 10000.0;
    double torque_limit_threshold = 0.0;
    double damping_coefficient = 0.0;
    double spring_coefficient = 0.0;
  };

 protected:
  /// This joint's name.
  std::string name_;

  /// ID reference to DynamicsSymbol.
  int id_ = -1;

  /// Joint frame defined in world frame.
  Pose3 wTj_;
  /// Rest transform to parent link CoM frame from joint frame.
  Pose3 jTpcom_;
  /// Rest transform to child link CoM frame from joint frame.
  Pose3 jTccom_;
  /// Rest transform to parent link com frame from child link com frame at rest.
  Pose3 pMccom_;

  LinkSharedPtr parent_link_;
  LinkSharedPtr child_link_;

  /// Joint parameters struct.
  Parameters parameters_;

  /// Abstract method. Return transform of child link com frame w.r.t parent
  /// link com frame
  gtsam::Pose3 pMcCom(boost::optional<double> q = boost::none);

  /// Abstract method. Return transform of parent link com frame w.r.t child
  /// link com frame
  gtsam::Pose3 cMpCom(boost::optional<double> q = boost::none);

  /// Check if the link is a child link, throw an error if link is not
  /// connected to this joint.
  bool isChildLink(const LinkSharedPtr &link) const {
    if (link != child_link_ && link != parent_link_)
      throw std::runtime_error("link " + link->name() +
                               " is not connected to this joint " + name_);
    return link == child_link_;
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
  Joint(const std::string &name, const Pose3 &wTj,
        const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
        const Parameters &parameters)
      : name_(name),
        wTj_(wTj),
        parent_link_(parent_link),
        child_link_(child_link),
        parameters_(parameters) {
    jTpcom_ = wTj_.inverse() * parent_link_->wTcom();
    jTccom_ = wTj_.inverse() * child_link_->wTcom();
    pMccom_ = parent_link_->wTcom().inverse() * child_link_->wTcom();
  }

  /**
   * @brief Default destructor.
   */
  virtual ~Joint() = default;

  /// Return a shared ptr to this joint.
  JointSharedPtr shared() { return shared_from_this(); }

  /// Return a const shared ptr to this joint.
  JointConstSharedPtr shared() const { return shared_from_this(); }

  /// Set the joint's ID.
  void setID(unsigned char id) { id_ = id; }

  /// Get the joint's ID.
  int id() const {
    if (id_ == -1)
      throw std::runtime_error(
          "Calling id on a joint whose ID has not been set");
    return id_;
  }

  /// Transform from the world frame to the joint frame.
  const Pose3 &wTj() const { return wTj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const Pose3 &jTpcom() const { return jTpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const Pose3 &jTccom() const { return jTccom_; }

  /// Get a gtsam::Key for this joint
  gtsam::Key key() const { return gtsam::Key(id()); }

  /// Return joint name.
  std::string name() const { return name_; }

  /// Return the connected link other than the one provided.
  LinkSharedPtr otherLink(const LinkSharedPtr &link) const {
    return isChildLink(link) ? parent_link_ : child_link_;
  }

  /// Return the links connected to this joint.
  std::vector<LinkSharedPtr> links() const {
    return std::vector<LinkSharedPtr>{parent_link_, child_link_};
  }

  /// Return a shared ptr to the parent link.
  LinkSharedPtr parent() const { return parent_link_; }

  /// Return a shared ptr to the child link.
  LinkSharedPtr child() const { return child_link_; }

  /// Return the ID of the parent link.
  int parentID() const { return parent_link_->id(); }

  /// Return the ID of the child link.
  int childID() const { return child_link_->id(); }

  /// Return the name of the parent link.
  std::string parentName() const { return parent_link_->name(); }

  /// Return the name of the child link.
  std::string childName() const { return child_link_->name(); }

  /// Return joint parameters.
  const Parameters &parameters() const { return parameters_; }

  bool operator==(const Joint &other) const {
    return (this->name_ == other.name_ && this->id_ == other.id_ &&
            this->wTj_.equals(other.wTj_) &&
            this->jTpcom_.equals(other.jTpcom_) &&
            this->jTccom_.equals(other.jTccom_) &&
            this->pMccom_.equals(other.pMccom_));
  }

  bool operator!=(const Joint &other) const { return !(*this == other); }

  friend std::ostream &operator<<(std::ostream &stream, const Joint &j);

  friend std::ostream &operator<<(std::ostream &stream,
                                  const JointSharedPtr &j);

  /// Helper print function
  void print() const { std::cout << *this; }

  /**
   * \defgroup AbstractMethods Abstract methods for the joint class.
   * @{
   */

  /**
   * Abstract method. Return the transform from the other link com to this link
   * com frame given a Values object containing this joint's angle Value
   */
  virtual Pose3 transformTo(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const = 0;

  /**
   * Abstract method: Return joint type for use in reconstructing robot from
   * Parameters.
   */
  virtual Type type() const = 0;

  /** Abstract method. Return the twist of the other link given this link's
   * twist and a Values object containing this joint's angle Value.
   */
  virtual gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_other_twist = boost::none) const = 0;

  /**
   * Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle Value and
   * derivatives.
   */
  virtual gtsam::Vector6 transformTwistAccelTo(
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
          boost::none) const = 0;

  /**
   * @fn Abstract method to return pose factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate q factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return pose factors.
   */
  virtual gtsam::NonlinearFactorGraph qFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /**
   * @fn (ABSTRACT) Return velocity factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate v factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return velocity factors.
   */
  virtual gtsam::NonlinearFactorGraph vFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /**
   * @fn (ABSTRACT) Return accel factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate a factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return accel factors.
   */
  virtual gtsam::NonlinearFactorGraph aFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /// Abstract method. Returns forward dynamics priors on torque
  virtual gtsam::GaussianFactorGraph linearFDPriors(
      size_t t, const JointValues &torques, const OptimizerSetting &opt) const {
    throw std::runtime_error(
        "linearFDPriors not implemented for the desired "
        "joint type.  A linearized version may not be possible.");
  }

  /**
   * @fn (ABSTRACT) Return linear accel factors in the dynamics graph.
   *
   * @param[in] t The timestep for which to generate factors.
   * @param[in] poses Link poses.
   * @param[in] twists Link twists.
   * @param[in] joint_angles Joint angles.
   * @param[in] joint_vels Joint velocities.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear accel factors.
   */
  virtual gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const JointValues &joint_angles, const JointValues &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const {
    throw std::runtime_error(
        "linearAFactors not implemented for the desired "
        "joint type.  A linearized version may not be possible.");
  }

  /**
   * @fn (ABSTRACT) Return dynamics factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate dynamics factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return dynamics factors.
   */
  virtual gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const = 0;

  /**
   * @fn (ABSTRACT) Return linear dynamics factors in the dynamics graph.
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
   * TODO(G+S): change angle/vel type from map<string, double> to gtsam::Values
   */
  virtual gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const {
    throw std::runtime_error(
        "linearDynamicsFactors not implemented for the "
        "desired joint type.  A linearized version may not be possible.");
  }

  /**
   * @fn (ABSTRACT) Return joint limit factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate joint limit factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return joint limit factors.
   */
  virtual gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const = 0;

  /**@}*/

  /**
   * Return the pose of this link com given a Values object containing this
   * joint's angle Value, and also given the other link's pose.
   * Equivalent to T_other.compose(transformFrom(link, q)).
   */
  Pose3 transformTo(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q,
      gtsam::Pose3 T_other, boost::optional<gtsam::Matrix &> H_q = boost::none,
      gtsam::OptionalJacobian<6, 6> H_T_other = boost::none) const {
    if (!H_q) {
      return T_other.compose(transformFrom(link, q), H_T_other);
    } else {
      gtsam::Matrix66 H_relPose;
      Pose3 error =
          T_other.compose(transformFrom(link, q, H_q), H_T_other, H_relPose);
      *H_q = H_relPose * (*H_q);
      return error;
    }
  }

  /**
   * Return the transform from this link com to the other link com frame given a
   * Values object containing this joint's angle value.
   */
  Pose3 transformFrom(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const {
    return transformTo(otherLink(link), q, H_q);
  }

  /**
   * Return the pose of other link com given a Values object containing other
   * joint's angle Value, and also given the this link's pose.
   * Equivalent to T_this.compose(transformTo(link, q))
   */
  Pose3 transformFrom(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q,
      gtsam::Pose3 T_this, boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_T_this = boost::none) const {
    return transformTo(otherLink(link), q, T_this, H_q, H_T_this);
  }

  /**
   * Return the twist of the other link given this link's twist and a Values
   * object containing this joint's angle value.
   */
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
      boost::optional<gtsam::Values> q_dot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_this_twist = boost::none) const {
    return transformTwistTo(otherLink(link), q, q_dot, this_twist, H_q, H_q_dot,
                            H_this_twist);
  }

  /**
   * Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle value and
   * derivatives.
   */
  gtsam::Vector6 transformTwistAccelFrom(
      const LinkSharedPtr &link, boost::optional<gtsam::Values> q = boost::none,
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

}  // namespace gtdynamics
