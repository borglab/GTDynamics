/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Robot.h
 * @brief Robot structure.
 * @author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <gtdynamics/config.h>
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/universal_robot/RobotTypes.h>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

/// Map from link name to link shared pointer
using LinkMap = std::map<std::string, LinkSharedPtr>;
/// Map from joint name to joint shared pointer
using JointMap = std::map<std::string, JointSharedPtr>;

using LinkVector = std::vector<std::reference_wrapper<Link>>;
using JointVector = std::vector<std::reference_wrapper<Joint>>;

using LinkJointPair = std::pair<LinkMap, JointMap>;
// map from link name to link pose
using LinkPoses = std::map<std::string, gtsam::Pose3>;
// map from link name to link twist
using LinkTwists = std::map<std::string, gtsam::Vector6>;
// type for storing forward kinematics results
using FKResults = std::pair<LinkPoses, LinkTwists>;

/**
 * Robot is used to create a representation of a robot's
 * inertial/dynamic properties from a URDF/SDF file. The resulting object
 * provides getters for the robot's various joints and links, which can then
 * be fed into an optimization pipeline.
 */
class Robot {
 private:
  // For quicker/easier access to links and joints.
  LinkMap name_to_link_;
  JointMap name_to_joint_;

 public:
  /** Default Constructor */
  Robot() {}

  /**
   * Constructor from link and joint elements.
   *
   * @param[in] links LinkMap containing all links
   * @param[in] joints JointMap containing all joints
   * joints.
   */
  explicit Robot(const LinkMap &links, const JointMap &joints);

  /// Return this robot's links.
  std::vector<LinkSharedPtr> links() const;

  /// Return this robot's joints.
  std::vector<JointSharedPtr> joints() const;

  /// remove specified link from the robot
  void removeLink(const LinkSharedPtr &link);

  /// remove specified joint from the robot
  void removeJoint(const JointSharedPtr &joint);

  /**
   * @brief Rename the links.
   *
   * @param name_map Map from old link name to new link name. This map must
   * contain entries for all links in the robot; if a link name is missing,
   * the implementation will throw (e.g., due to use of std::map::at()).
   */
  void renameLinks(const std::map<std::string, std::string>& name_map);

  /**
   * @brief Rename the joints.
   *
   * The provided map must contain an entry for every joint in the robot.
   * Internally, this function uses std::map::at() to look up the new name
   * for each existing joint name, so missing keys will result in an
   * exception being thrown.
   *
   * @param name_map Map from old joint name to new joint name, containing
   *                 entries for all joints in the robot.
   */
  void renameJoints(const std::map<std::string, std::string>& name_map);

  /**
   * @brief Reassign the link IDs.
   *
   * The provided list must contain the names of all links in the robot,
   * with no omissions or duplicates. Internally the implementation uses
   * a map lookup that will throw if any name is missing. The index of
   * each name in the vector (0-indexed) determines the new ID assigned
   * to the corresponding link.
   *
   * @param ordered_link_names List of all link names in the desired new
   *        ID order (0-indexed).
   */
  void reassignLinks(const std::vector<std::string>& ordered_link_names);

  /**
   * @brief Reassign the joint IDs.
   *
   * @param ordered_joint_names List of joint names in the desired new order.
   *        This vector must contain the names of all joints in the robot,
   *        each appearing exactly once. The index of each name in the vector
   *        determines the joint's new ID (0-indexed).
   */
  void reassignJoints(const std::vector<std::string>& ordered_joint_names);

  /**
   * @brief Return links ordered by their IDs.
   *
   * @return std::vector<LinkSharedPtr>
   */
  std::vector<LinkSharedPtr> orderedLinks() const;

  /**
   * @brief Return joints ordered by their IDs.
   *
   * @return std::vector<JointSharedPtr>
   */
  std::vector<JointSharedPtr> orderedJoints() const;

  /// Return the link corresponding to the input string.
  LinkSharedPtr link(const std::string &name) const;

  /**
   * @brief Return a copy of this robot with the link corresponding to the input
   * string as a fixed link.
   *
   * @param name The name of the link to fix.
   * @return Robot
   */
  Robot fixLink(const std::string &name) const;

  /**
   * @brief Return a copy of this robot after unfixing the link corresponding to
   * the input string.
   *
   * @param name The name of the link to unfix.
   * @return Robot
   */
  Robot unfixLink(const std::string &name) const;

  /// Return the joint corresponding to the input string.
  JointSharedPtr joint(const std::string &name) const;

  /// Return number of *moving* links.
  int numLinks() const;

  /// Return number of joints.
  int numJoints() const;

  /// Print links and joints of the robot, for debug purposes
  void print(const std::string &s = "") const;

  /// Overload equality operator.
  bool operator==(const Robot &other) const {
    // Define comparators for easy std::map equality checking
    // Needed since we are storing shared pointers as the values.
    auto link_comparator = [](decltype(*this->name_to_link_.begin()) a,
                              decltype(a) b) {
      // compare the key name and the underlying shared_ptr object
      return a.first == b.first && (*a.second) == (*b.second);
    };
    auto joint_comparator = [](decltype(*this->name_to_joint_.begin()) a,
                               decltype(a) b) {
      // compare the key name and the underlying shared_ptr object
      return a.first == b.first && (*a.second) == (*b.second);
    };

    return (this->name_to_link_.size() == other.name_to_link_.size() &&
            std::equal(this->name_to_link_.begin(), this->name_to_link_.end(),
                       other.name_to_link_.begin(), link_comparator) &&
            this->name_to_joint_.size() == other.name_to_joint_.size() &&
            std::equal(this->name_to_joint_.begin(), this->name_to_joint_.end(),
                       other.name_to_joint_.begin(), joint_comparator));
  }

  bool equals(const Robot &other, double tol = 0) const {
    return *this == other;
  }

  /**
   * Calculate forward kinematics by performing BFS in the link-joint graph
   * (will throw an error when invalid joint angle specification detected).
   *
   * If the root link pose and twist are not provided in `known_values`,
   * default Pose3() and Vector6::Zeros() are used respectively.
   *
   * *Important* Forward kinematics is done with CoM frames.
   *
   * @param[in] t integer time index
   * @param[in] known_values Values with joint angles, joint velocities, and
   * (optionally) root link pose and twist.
   * @param[in] prior_link_name name of link with known pose & twist
   * @return CoM poses and twists of all links, as a new Values instance
   */
  gtsam::Values forwardKinematics(
      const gtsam::Values &known_values, size_t t = 0,
      const std::optional<std::string> &prior_link_name = {}) const;

 private:
  /// Find root link for forward kinematics
  LinkSharedPtr findRootLink(
      const gtsam::Values &values,
      const std::optional<std::string> &prior_link_name) const;

  /// @name Advanced Interface
  /// @{

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(name_to_link_);
    ar &BOOST_SERIALIZATION_NVP(name_to_joint_);
  }
#endif

  /// @}
};
}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::Robot> : public Testable<gtdynamics::Robot> {};

}  // namespace gtsam
