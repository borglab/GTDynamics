/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.h
 * @brief Absract representation of a robot joint.
 * @author: Frank Dellaert
 * @author: Mandy Xie
 * @author: Alejandro Escontrela
 * @author: Yetong Zhang
 * @author: Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/utils/DynamicsSymbol.h"

namespace gtdynamics {

class Joint;  // forward declaration
class Link;   // forward declaration

LINK_TYPEDEF_CLASS_POINTER(Link);
LINK_TYPEDEF_CLASS_POINTER(Joint);

enum JointEffortType { Actuated, Unactuated, Impedance };

/**
 * This struct contains information for scalar limits.
 * The lower and upper limits denote physical axis limits of the joint,
 * and the threshold is an error threshold used in calculations.
 */
struct JointScalarLimit {
  double value_lower_limit = -M_PI_2;
  double value_upper_limit = M_PI_2;
  double value_limit_threshold = 1e-9;
};

/**
 * This struct contains all parameters needed to construct a joint.
 */
struct JointParams {
  JointEffortType effort_type = JointEffortType::Actuated;
  JointScalarLimit scalar_limits;
  double velocity_limit = 10000.0;
  double velocity_limit_threshold = 0.0;
  double acceleration_limit = 10000.0;
  double acceleration_limit_threshold = 0.0;
  double torque_limit = 10000.0;
  double torque_limit_threshold = 0.0;
  double damping_coefficient = 0.0;
  double spring_coefficient = 0.0;

  /// Constructor
  JointParams() {}
};

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

  enum Type : char {
    Revolute = 'R',
    Prismatic = 'P',
    Screw = 'C',
    ScrewAxis = 'A'
  };

 protected:
  /// This joint's name.
  std::string name_;

  /// ID reference to DynamicsSymbol.
  uint8_t id_;

  /// Joint frame defined in world frame.
  Pose3 wTj_;
  /// Rest transform to parent link CoM frame from joint frame.
  Pose3 jTpcom_;
  /// Rest transform to child link CoM frame from joint frame.
  Pose3 jTccom_;
  /// Transform to parent link com frame from child link com frame at rest.
  Pose3 pMccom_;

  using LinkSharedPtr = boost::shared_ptr<Link>;
  LinkSharedPtr parent_link_;
  LinkSharedPtr child_link_;

  /// Joint parameters struct.
  JointParams parameters_;

  /// Check if the link is a child link, throw an error if link is not
  /// connected to this joint.
  bool isChildLink(const LinkSharedPtr &link) const;

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
   * @param[in] parameters   The joint parameters.
   */
  Joint(uint8_t id, const std::string &name, const Pose3 &wTj,
        const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
        const JointParams &parameters = JointParams());

  /**
   * @brief Default destructor.
   */
  virtual ~Joint() = default;

  /// Return a shared ptr to this joint.
  JointSharedPtr shared() { return shared_from_this(); }

  /// Return a const shared ptr to this joint.
  JointConstSharedPtr shared() const { return shared_from_this(); }

  /// Get the joint's ID.
  uint8_t id() const { return id_; }

  /// Transform from the world frame to the joint frame.
  const Pose3 &wTj() const { return wTj_; }

  /// Transform from the joint frame to the parent's center of mass.
  const Pose3 &jTpcom() const { return jTpcom_; }

  /// Transform from the joint frame to the child's center of mass.
  const Pose3 &jTccom() const { return jTccom_; }

  /// Get a gtsam::Key for this joint
  gtsam::Key key() const { return gtsam::Key(id()); }

  /// Return joint name.
  const std::string &name() const { return name_; }

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

  /// Return joint parameters.
  const JointParams &parameters() const { return parameters_; }

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
  void print(const std::string &s = "") const {
    std::cout << (s.empty() ? s : s + " ") << *this;
  }

  /**
   * \defgroup AbstractMethods Abstract methods for the joint class.
   * @{
   */

  /**
   * Abstract method: Return joint type for use in reconstructing robot from
   * JointParams.
   */
  virtual Type type() const = 0;

  /**
   * Abstract method. Return the pose of the child link CoM in the parent link
   * CoM frame, given a Values object containing the joint coordinate.
   */
  virtual Pose3 parentTchild(
      const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const = 0;

  /**
   * Abstract method. Return the pose of the parent link in the child link
   * frame, given a Values object containing the joint coordinate.
   */
  virtual Pose3 childTparent(
      const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const = 0;

  /**
   * Abstract method. Return the relative pose of the specified link [link2] in
   * the other link's [link1] reference frame.
   */
  virtual Pose3 relativePoseOf(
      const LinkSharedPtr &link2, const gtsam::Values &q, size_t t = 0,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const = 0;

  /**
   * Return the world pose of the specified link [link2], given
   * the world pose of the other link [link1].
   */
  Pose3 poseOf(const LinkSharedPtr &link2, const Pose3 &wT1,
               const gtsam::Values &q, size_t t = 0,
               gtsam::OptionalJacobian<6, 6> H_wT1 = boost::none,
               boost::optional<gtsam::Matrix &> H_q = boost::none) const {
    auto T12 = relativePoseOf(link2, q, t, H_q);
    return wT1.compose(T12, H_wT1);  // H_wT2_T12 is identity
  }

  /** Abstract method. Return the twist of the other link given this link's
   * twist and a Values object containing this joint's angle Value.
   */
  virtual gtsam::Vector6 transformTwistTo(
      size_t t, const LinkSharedPtr &link, const gtsam::Values &q_and_q_dot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const = 0;

  /**
   * Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle Value and
   * derivatives.
   */
  virtual gtsam::Vector6 transformTwistAccelTo(
      size_t t, const LinkSharedPtr &link,
      const gtsam::Values &q_and_q_dot_and_q_ddot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_q_dot = boost::none,
      boost::optional<gtsam::Matrix &> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const = 0;

  /// Abstract method. Returns forward dynamics priors on torque
  virtual gtsam::GaussianFactorGraph linearFDPriors(
      size_t t, const gtsam::Values &torques,
      const OptimizerSetting &opt) const {
    throw std::runtime_error(
        "linearFDPriors not implemented for the desired "
        "joint type.  A linearized version may not be possible.");
  }

  /**
   * @fn (ABSTRACT) Return linear accel factors in the dynamics graph.
   *
   * @param[in] t The timestep for which to generate factors.
   * @param[in] known_values Link poses, twists, Joint angles, Joint velocities.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear accel factors.
   */
  virtual gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const {
    throw std::runtime_error(
        "linearAFactors not implemented for the desired "
        "joint type.  A linearized version may not be possible.");
  }

  /**
   * @fn (ABSTRACT) Return linear dynamics factors in the dynamics graph.
   *
   * @param[in] t             The timestep for which to generate factors.
   * @param[in] known_values  Link poses, twists, Joint angles, Joint
   * velocities.
   * @param[in] opt           OptimizerSetting object containing NoiseModels
   *    for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear dynamics factors.
   */
  virtual gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const {
    throw std::runtime_error(
        "linearDynamicsFactors not implemented for the "
        "desired joint type.  A linearized version may not be possible.");
  }

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

  /// Joint-induced twist in child frame
  virtual gtsam::Vector6 childTwist(const gtsam::Values &values,
                                    size_t t = 0) const = 0;

  /// Joint-induced twist in parent frame
  virtual gtsam::Vector6 parentTwist(const gtsam::Values &values,
                                     size_t t = 0) const = 0;

  /// Calculate pose/twist of child given parent pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> childPoseTwist(
      const gtsam::Pose3 &wTp, const gtsam::Vector6 &Vp,
      const gtsam::Values &known_values, size_t t = 0) const {
    const gtsam::Pose3 pTc = parentTchild(known_values, t);
    return {wTp * pTc, pTc.inverse().Adjoint(Vp) + childTwist(known_values, t)};
  }

  /// Calculate pose/twist of parent given child pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> parentPoseTwist(
      const gtsam::Pose3 &wTc, const gtsam::Vector6 &Vc,
      const gtsam::Values &known_values, size_t t = 0) const {
    const gtsam::Pose3 pTc = parentTchild(known_values, t);
    return {wTc * pTc.inverse(),
            pTc.Adjoint(Vc) + parentTwist(known_values, t)};
  }

  /// Given link pose/twist, calculate pose/twist of other link
  std::pair<gtsam::Pose3, gtsam::Vector6> otherPoseTwist(
      const LinkSharedPtr &link, const gtsam::Pose3 &wTl,
      const gtsam::Vector6 &Vl, const gtsam::Values &known_values,
      size_t t = 0) const {
    return isChildLink(link) ? parentPoseTwist(wTl, Vl, known_values, t)
                             : childPoseTwist(wTl, Vl, known_values, t);
  }
};

}  // namespace gtdynamics
