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

/**
 * Joint effort types
 *
 * Actuated: motor powered
 * Unactuated: not powered, free to move, exert zero torque
 * Impedance: with spring resistance
 */
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

  /// Robot class should have access to the internals of its joints.
  friend class Robot;

 protected:
  using Pose3 = gtsam::Pose3;
  using Vector6 = gtsam::Vector6;

 public:
  /// Joint Type (see Lee09mmt, 2.1 paragraph 2)
  enum Type : char {
    Revolute = 'R',
    Prismatic = 'P',
    Screw = 'H',
  };

 protected:
  /// This joint's name.
  std::string name_;

  /// ID reference to DynamicsSymbol.
  uint8_t id_;

  /// Rest transform to parent link CoM frame from joint frame.
  Pose3 jMp_;
  /// Rest transform to child link CoM frame from joint frame.
  Pose3 jMc_;

  using LinkSharedPtr = boost::shared_ptr<Link>;
  LinkSharedPtr parent_link_;
  LinkSharedPtr child_link_;

  // Screw axis in parent and child CoM frames.
  Vector6 pScrewAxis_;
  Vector6 cScrewAxis_;

  /// Joint parameters struct.
  JointParams parameters_;

  /// Check if the link is a child link, throw an error if link is not
  /// connected to this joint.
  bool isChildLink(const LinkSharedPtr &link) const;

 public:
  Joint() {}

  /**
   * @brief Constructor to create Joint from joint name, joint pose in
   * base frame, and shared pointers to the parent and child links.
   *
   * @param[in] name         name of joint
   * @param[in] bTj          joint pose expressed in base frame
   * @param[in] parent_link  Shared pointer to the parent Link.
   * @param[in] child_link   Shared pointer to the child Link.
   * @param[in] jScrewAxis   Screw axis in the joint frame
   * @param[in] parameters   The joint parameters.
   */
  Joint(uint8_t id, const std::string &name, const Pose3 &bTj,
        const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
        const Vector6 &jScrewAxis,
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

  /// Transform from the joint frame to the parent's center of mass.
  const Pose3 &jMp() const { return jMp_; }

  /// Transform from the joint frame to the child's center of mass.
  const Pose3 &jMc() const { return jMc_; }

  /// Return screw axis expressed in the specified link frame
  const Vector6 screwAxis(const LinkSharedPtr &link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

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
            this->jMp_.equals(other.jMp_) && this->jMc_.equals(other.jMc_));
  }

  bool operator!=(const Joint &other) const { return !(*this == other); }

  friend std::ostream &operator<<(std::ostream &stream, const Joint &j);

  friend std::ostream &operator<<(std::ostream &stream,
                                  const JointSharedPtr &j);

  /// Helper print function
  void print(const std::string &s = "") const {
    std::cout << (s.empty() ? s : s + " ") << *this;
  }

  /// Helper function for overloading stream operator
  virtual std::ostream &to_stream(std::ostream &os) const;

  /**
   * \defgroup AbstractMethods Abstract methods for the joint class.
   * @{
   */

  /**
   * Abstract method: Return joint type for use in reconstructing robot from
   * JointParams.
   */
  virtual Type type() const = 0;

  /**@}*/

  /**
   * Return transform of child link CoM frame w.r.t parent link CoM frame
   */
  Pose3 parentTchild(double q,
                     gtsam::OptionalJacobian<6, 1> pMc_H_q = boost::none) const;

  /**
   * Return transform of parent link CoM frame w.r.t child link CoM frame
   */
  Pose3 childTparent(double q,
                     gtsam::OptionalJacobian<6, 1> cMp_H_q = boost::none) const;

  /**
   * Return the relative pose of the specified link [link2] in the other link's
   * [link1] reference frame.
   */
  Pose3 relativePoseOf(const LinkSharedPtr &link2, double q,
                       gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    return isChildLink(link2) ? parentTchild(q, H_q) : childTparent(q, H_q);
  }

  /**
   * Return the world pose of the specified link [link2], given
   * the world pose of the other link [link1].
   */
  Pose3 poseOf(const LinkSharedPtr &link2, const Pose3 &wT1, double q,
               gtsam::OptionalJacobian<6, 6> H_wT1 = boost::none,
               gtsam::OptionalJacobian<6, 1> H_q = boost::none) const {
    auto T12 = relativePoseOf(link2, q, H_q);
    return wT1.compose(T12, H_wT1);  // H_wT2_T12 is identity
  }

  /** Abstract method. Return the twist of the other link given this link's
   * twist and a Values object containing this joint's angle Value.
   */
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, double q, double q_dot,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const;

  /**
   * Return the twist acceleration of the other link given this link's twist
   * accel and a Values object containing this joint's angle Value and
   * derivatives.
   */
  gtsam::Vector6 transformTwistAccelTo(
      const LinkSharedPtr &link, double q, double q_dot, double q_ddot,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel = boost::none) const;

  /// Return the torque on this joint given the wrench
  double transformWrenchToTorque(
      const LinkSharedPtr &link,
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<1, 6> H_wrench = boost::none) const;

  /// Returns forward dynamics priors on torque
  gtsam::GaussianFactorGraph linearFDPriors(size_t t,
                                            const gtsam::Values &torques,
                                            const OptimizerSetting &opt) const;

  /**
   * Returns linear acceleration factors in the dynamics graph.
   *
   * @param[in] t The timestep for which to generate factors.
   * @param[in] known_values Link poses, twists, Joint angles, Joint velocities.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear accel factors.
   */
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /**
   * Returns linear dynamics factors in the dynamics graph.
   *
   * @param[in] t             The timestep for which to generate factors.
   * @param[in] known_values  Link poses, twists, Joint angles, Joint
   * velocities.
   * @param[in] opt           OptimizerSetting object containing NoiseModels
   *    for factors.
   * @param[in] planar_axis   Optional planar axis.
   * @return linear dynamics factors.
   */
  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /**
   * @fn (ABSTRACT) Return joint limit factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate joint limit factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return joint limit factors.
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const;

  /// Joint-induced twist in child frame
  Vector6 childTwist(double q_dot) const;

  /// Joint-induced twist in parent frame
  Vector6 parentTwist(double q_dot) const;

  /// Calculate pose/twist of child given parent pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> childPoseTwist(
      const gtsam::Pose3 &wTp, const gtsam::Vector6 &Vp,
      double q, double q_dot) const {
    const gtsam::Pose3 pTc = parentTchild(q);
    return {wTp * pTc, pTc.inverse().Adjoint(Vp) + childTwist(q_dot)};
  }

  /// Calculate pose/twist of parent given child pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> parentPoseTwist(
      const gtsam::Pose3 &wTc, const gtsam::Vector6 &Vc, double q,
      double q_dot) const {
    const gtsam::Pose3 pTc = parentTchild(q);
    return {wTc * pTc.inverse(),
            pTc.Adjoint(Vc) + parentTwist(q_dot)};
  }

  /// Given link pose/twist, calculate pose/twist of other link
  std::pair<gtsam::Pose3, gtsam::Vector6> otherPoseTwist(
      const LinkSharedPtr &link, const gtsam::Pose3 &wTl,
      const gtsam::Vector6 &Vl, double q, double q_dot) const {
    return isChildLink(link) ? parentPoseTwist(wTl, Vl, q, q_dot)
                             : childPoseTwist(wTl, Vl, q, q_dot);
  }
};

using JointTyped = Joint;

}  // namespace gtdynamics
