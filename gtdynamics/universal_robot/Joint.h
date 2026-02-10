/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.h
 * @brief Abstract representation of a robot joint.
 * @author: Frank Dellaert
 * @author: Mandy Xie
 * @author: Alejandro Escontrela
 * @author: Yetong Zhang
 * @author: Varun Agrawal
 */

#pragma once

#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/universal_robot/RobotTypes.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#endif

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

  JointScalarLimit() {}

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(value_lower_limit);
    ar &BOOST_SERIALIZATION_NVP(value_upper_limit);
    ar &BOOST_SERIALIZATION_NVP(value_limit_threshold);
  }
#endif
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

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(effort_type);
    ar &BOOST_SERIALIZATION_NVP(scalar_limits);
    ar &BOOST_SERIALIZATION_NVP(velocity_limit);
    ar &BOOST_SERIALIZATION_NVP(velocity_limit_threshold);
    ar &BOOST_SERIALIZATION_NVP(acceleration_limit);
    ar &BOOST_SERIALIZATION_NVP(acceleration_limit_threshold);
    ar &BOOST_SERIALIZATION_NVP(torque_limit);
    ar &BOOST_SERIALIZATION_NVP(torque_limit_threshold);
    ar &BOOST_SERIALIZATION_NVP(damping_coefficient);
    ar &BOOST_SERIALIZATION_NVP(spring_coefficient);
  }
#endif
};

/// Joint is the base class for a joint connecting two Link objects.
class Joint : public std::enable_shared_from_this<Joint> {
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
    Fixed = 'F',
  };

  static std::string JointTypeString(const Type &type) {
    std::map<Type, std::string> joint_type_map = {
        {Revolute, "Revolute"},
        {Prismatic, "Prismatic"},
        {Screw, "Screw"},
        {Fixed, "Fixed"},
    };
    return joint_type_map[type];
  }

 protected:
  /// This joint's name.
  std::string name_;

  /// ID reference to DynamicsSymbol.
  uint8_t id_;

  Pose3 pMj_;  ///< Joint frame in parent link CoM frame at rest.
  Pose3 jMc_;  ///< Child CoM frame in joint  frame at rest.
  Pose3 pMc_;  ///< Child CoM frame in parent link CoM frame at rest.

  // NOTE: We use a weak_ptr here since Link has shared_ptrs
  // to joints, and this way we can avoid reference cycles.
  // https://en.cppreference.com/w/cpp/memory/weak_ptr
  using LinkWeakPtr = std::weak_ptr<Link>;
  LinkWeakPtr parent_link_;
  LinkWeakPtr child_link_;

  // Screw axis in parent and child CoM frames.
  Vector6 pScrewAxis_;
  Vector6 cScrewAxis_;

  /// Joint parameters struct.
  JointParams parameters_;

  /// Check if the link is a child link, throw an error if link is not
  /// connected to this joint.
  bool isChildLink(const LinkSharedPtr &link) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
        const LinkWeakPtr &parent_link, const LinkWeakPtr &child_link,
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

  /// Return (unchanging) pose of the joint frame in parent link CoM frame.
  const Pose3 &pMj() const { return pMj_; }

  /// Return (unchanging) pose of the child link's COM in the joint frame.
  const Pose3 &jMc() const { return jMc_; }

  /// Return pose of child CoM frame in parent link CoM frame, at rest.
  const Pose3 &pMc() const { return pMc_; }

  /// Return screw axis in parent link frame.
  const Vector6 &pScrewAxis() const { return pScrewAxis_; }

  /// Return screw axis in child link frame.
  const Vector6 &cScrewAxis() const { return cScrewAxis_; }

  /// Return screw axis expressed in the specified link frame.
  Vector6 screwAxis(const LinkSharedPtr &link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

  /// Get a gtsam::Key for this joint
  gtsam::Key key() const { return gtsam::Key(id()); }

  /// Return joint name.
  const std::string &name() const { return name_; }

  /**
   * @brief Rename the joint.
   *
   * @param new_name The new name of the joint.
   */
  void rename(const std::string &new_name) { name_ = new_name; }

  /**
   * @brief Reassign the joint ID.
   *
   * @param new_id The new ID of the joint.
   */
  void reassign(const uint8_t new_id) { id_ = new_id; }

  /// Return the connected link other than the one provided.
  LinkSharedPtr otherLink(const LinkSharedPtr &link) const {
    return isChildLink(link) ? parent_link_.lock() : child_link_.lock();
  }

  /// Return the links connected to this joint.
  std::vector<LinkSharedPtr> links() const {
    return std::vector<LinkSharedPtr>{parent_link_.lock(), child_link_.lock()};
  }

  /// Return a shared ptr to the parent link.
  LinkSharedPtr parent() const { return parent_link_.lock(); }

  /// Return a shared ptr to the child link.
  LinkSharedPtr child() const { return child_link_.lock(); }

  /// Return joint parameters.
  const JointParams &parameters() const { return parameters_; }

  bool operator==(const Joint &other) const {
    return (this->name_ == other.name_ && this->id_ == other.id_ &&
            this->pMj_.equals(other.pMj_) && this->jMc_.equals(other.jMc_) &&
            this->pMc_.equals(other.pMc_));
  }

  bool operator!=(const Joint &other) const { return !(*this == other); }

  bool equals(const Joint &other, double tol = 0) const {
    return *this == other;
  }

  friend std::ostream &operator<<(std::ostream &stream, const Joint &j);

  friend std::ostream &operator<<(std::ostream &stream,
                                  const JointSharedPtr &j);

  /// Helper print function
  void print(const std::string &s = "") const {
    std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
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
                     gtsam::OptionalJacobian<6, 1> pMc_H_q = {}) const;

  /**
   * Return transform of parent link CoM frame w.r.t child link CoM frame
   */
  Pose3 childTparent(double q,
                     gtsam::OptionalJacobian<6, 1> cMp_H_q = {}) const;

  /**
   * Return the relative pose of the specified link [link2] in the other link's
   * [link1] reference frame.
   */
  Pose3 relativePoseOf(const LinkSharedPtr &link2, double q,
                       gtsam::OptionalJacobian<6, 1> H_q = {}) const {
    return isChildLink(link2) ? parentTchild(q, H_q) : childTparent(q, H_q);
  }

  /**
   * Return the world pose of the specified link [link2], given
   * the world pose of the other link [link1].
   */
  Pose3 poseOf(const LinkSharedPtr &link2, const Pose3 &wT1, double q,
               gtsam::OptionalJacobian<6, 6> H_wT1 = {},
               gtsam::OptionalJacobian<6, 1> H_q = {}) const {
    auto T12 = relativePoseOf(link2, q, H_q);
    return wT1.compose(T12, H_wT1);  // H_wT2_T12 is identity
  }

  /** Abstract method. Return the twist of the other link given this link's
   * twist and a Values object containing this joint's angle Value.
   */
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr &link, double q, double q_dot,
      std::optional<gtsam::Vector6> other_twist = {},
      gtsam::OptionalJacobian<6, 1> H_q = {},
      gtsam::OptionalJacobian<6, 1> H_q_dot = {},
      gtsam::OptionalJacobian<6, 6> H_other_twist = {}) const;

  /**
   * Express the same wrench in the coordinate frame of the other link. (This
   * function is used for wrench equivalence constraint.)
   */
  gtsam::Vector6 transformWrenchCoordinate(
      const LinkSharedPtr &link, double q, const gtsam::Vector6 &wrench,
      gtsam::OptionalJacobian<6, 1> H_q = {},
      gtsam::OptionalJacobian<6, 6> H_wrench = {}) const;

  /// Return the torque on this joint given the wrench
  double transformWrenchToTorque(
      const LinkSharedPtr &link, std::optional<gtsam::Vector6> wrench = {},
      gtsam::OptionalJacobian<1, 6> H_wrench = {}) const;

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
      const std::optional<gtsam::Vector3> &planar_axis = {}) const;

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
      const std::optional<gtsam::Vector3> &planar_axis = {}) const;

  /**
   * Return joint limit factors in the dynamics graph.
   *
   * @param[in] t   The timestep for which to generate joint limit factors.
   * @param[in] opt OptimizerSetting object containing NoiseModels for factors.
   * @return joint limit factors.
   *
   * TODO(gerry): remove this out of Joint and into DynamicsGraph
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const;

  /// Joint-induced twist in child frame
  Vector6 childTwist(double q_dot) const;

  /// Joint-induced twist in parent frame
  Vector6 parentTwist(double q_dot) const;

  /// Calculate pose/twist of child given parent pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> childPoseTwist(
      const gtsam::Pose3 &wTp, const gtsam::Vector6 &Vp, double q,
      double q_dot) const {
    const gtsam::Pose3 pTc = parentTchild(q);
    return {wTp * pTc, pTc.inverse().Adjoint(Vp) + childTwist(q_dot)};
  }

  /// Calculate pose/twist of parent given child pose/twist
  std::pair<gtsam::Pose3, gtsam::Vector6> parentPoseTwist(
      const gtsam::Pose3 &wTc, const gtsam::Vector6 &Vc, double q,
      double q_dot) const {
    const gtsam::Pose3 pTc = parentTchild(q);
    return {wTc * pTc.inverse(), pTc.Adjoint(Vc) + parentTwist(q_dot)};
  }

  /// Given link pose/twist, calculate pose/twist of other link
  std::pair<gtsam::Pose3, gtsam::Vector6> otherPoseTwist(
      const LinkSharedPtr &link, const gtsam::Pose3 &wTl,
      const gtsam::Vector6 &Vl, double q, double q_dot) const {
    return isChildLink(link) ? parentPoseTwist(wTl, Vl, q, q_dot)
                             : childPoseTwist(wTl, Vl, q, q_dot);
  }

  /**
   * @brief Create expression that constraint the pose of two links imposed by
   * the joint angle.
   */
  gtsam::Vector6_ poseConstraint(uint64_t t = 0) const;

  /**
   * @brief Create expression that constraint the pose of two links imposed by
   * the joint angle. Alternative version using custom keys.
   * TODO(Varun) Need to do the same for all other constraints below
   */
  gtsam::Vector6_ poseConstraint(const DynamicsSymbol &wTp_key,
                                 const DynamicsSymbol &wTc_key,
                                 const DynamicsSymbol &q_key) const;

  /**
   * @brief Create expression that constraint the twist of two links imposed by
   * the joint angle and velocity.
   */
  gtsam::Vector6_ twistConstraint(uint64_t t = 0) const;

  /**
   * @brief Create expression that constraint the twist acceleration of two
   * links imposed by the joint angle, velocity and acceleration.
   */
  gtsam::Vector6_ twistAccelConstraint(uint64_t t = 0) const;

  /**
   * @brief Create expression that constraint the relation between wrench
   * expressed in two link frames.
   */
  gtsam::Vector6_ wrenchEquivalenceConstraint(uint64_t t = 0) const;

  /**
   * @brief Create expression for child wrench adjoint from parent link
   */
  gtsam::Vector6_ childWrenchAdjoint(gtsam::Vector6_ &wrench_p,
                                     uint64_t t = 0) const;

  /**
   * @brief Create expression that constraint the relation between
   * wrench and torque on each link.
   */
  gtsam::Double_ torqueConstraint(uint64_t t = 0) const;

 private:
  /// @name Advanced Interface
  /// @{

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(name_);
    ar &BOOST_SERIALIZATION_NVP(id_);
    ar &BOOST_SERIALIZATION_NVP(pMj_);
    ar &BOOST_SERIALIZATION_NVP(jMc_);
    ar &BOOST_SERIALIZATION_NVP(pMc_);
    ar &BOOST_SERIALIZATION_NVP(parent_link_);
    ar &BOOST_SERIALIZATION_NVP(child_link_);
    ar &BOOST_SERIALIZATION_NVP(pScrewAxis_);
    ar &BOOST_SERIALIZATION_NVP(cScrewAxis_);
    ar &BOOST_SERIALIZATION_NVP(parameters_);
  }
#endif

  /// @}
};

}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::Joint> : public Testable<gtdynamics::Joint> {};

}  // namespace gtsam
