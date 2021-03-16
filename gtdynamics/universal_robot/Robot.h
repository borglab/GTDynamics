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

#include <boost/optional.hpp>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

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
  explicit Robot(const LinkMap& links, const JointMap& joints);

  /// Return this robot's links.
  std::vector<LinkSharedPtr> links() const;

  /// Return this robot's joints.
  std::vector<JointSharedPtr> joints() const;

  /// remove specified link from the robot
  void removeLink(const LinkSharedPtr &link);

  /// remove specified joint from the robot
  void removeJoint(JointSharedPtr joint);

  /// Return the link corresponding to the input string.
  LinkSharedPtr link(const std::string &name) const;

  /// Fix the link corresponding to the input string.
  Robot fixLink(const std::string &name);

  /// Return the joint corresponding to the input string.
  JointSharedPtr joint(const std::string &name) const;

  /// Return number of *moving* links.
  int numLinks() const;

  /// Return number of joints.
  int numJoints() const;

  /// Print links and joints of the robot, for debug purposes
  void print() const;

  /**
   * Calculate forward kinematics by performing BFS in the link-joint graph.
   * 
   * (Will throw an error when invalid joint angle specification detected).
   *
   * @param[in] joint_angles joint angles for all joints in radians
   * @param[in] joint_vels joint velocities for all joints
   * @param[in] prior_link_name name of link with known pose & twist
   * @param[in] prior_link_pose pose of the known link
   * @param[in] prior_link_twist twist of the konwn link
   * @return poses and twists of all links
   */
  FKResults forwardKinematics(
      const JointValues &joint_angles, const JointValues &joint_vels,
      const boost::optional<std::string> prior_link_name = boost::none,
      const gtsam::Pose3 &prior_link_pose = gtsam::Pose3(),
      const gtsam::Vector6 &prior_link_twist = gtsam::Z_6x1) const;
};
}  // namespace gtdynamics
