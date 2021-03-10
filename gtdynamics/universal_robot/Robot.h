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
   * @param[in] robot_links_and_joints LinkJointPair containing links and
   * joints.
   */
  explicit Robot(LinkJointPair links_and_joints);

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
   * Calculate forward kinematics by performing bfs in the link-joint graph
   * (will throw an error when invalid joint angle specification detected).
   *
   * @param[in] joint_angles joint angles for all joints
   * @param[in] joint_vels joint velocities for all joints
   * @param[in] prior_link_name name of link with known pose & twist
   * @param[in] prior_link_pose pose of the known link
   * @param[in] prior_link_twist twist of the konwn link
   * @return poses and twists of all links
   */
  FKResults forwardKinematics(
      const JointValues &joint_angles, const JointValues &joint_vels,
      const boost::optional<std::string> prior_link_name = boost::none,
      const boost::optional<gtsam::Pose3> &prior_link_pose = boost::none,
      const boost::optional<gtsam::Vector6> &prior_link_twist =
          boost::none) const;

  /**
   * Returns pose factors for the robot.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] opt OptimizerSetting object.
   * @return pose factors.
   */
  gtsam::NonlinearFactorGraph qFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /**
   * Returns velocity factors for the robot.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] opt OptimizerSetting object.
   * @return velocity factors.
   */
  gtsam::NonlinearFactorGraph vFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /**
   * Returns acceleration factors for the robot.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] opt OptimizerSetting object.
   * @return acceleration factors.
   */
  gtsam::NonlinearFactorGraph aFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /**
   * Returns linear forward dynamics priors for the robot.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] torques Joint torques.
   * @param[in] opt OptimizerSetting object.
   * @return Linear forward dynamics priors.
   */
  gtsam::GaussianFactorGraph linearFDPriors(size_t t,
                                            const JointValues &torques,
                                            const OptimizerSetting &opt) const;

  /**
   * Returns accel factors linearized about specified operating  condition.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] poses Link poses.
   * @param[in] twists Link twists.
   * @param[in] joint_angles Joint angles.
   * @param[in] joint_vels Joint velocities.
   * @param[in] opt OptimizerSetting object.
   * @param[in] planar_axis Optional planar axis.
   * @return Linearized accel factors.
   */
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /**
   * Returns dynamics factors for the robot.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] opt OptimizerSetting object.
   * @param[in] planar_axis Optional planar axis.
   * @return dynamics factors.
   */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const;

  /**
   * Returns dynamics factors linearized about specified operating condition.
   *
   * @param[in] t Timestep to return q factors for.
   * @param[in] poses Link poses.
   * @param[in] twists Link twists.
   * @param[in] joint_angles Joint angles.
   * @param[in] joint_vels Joint velocities.
   * @param[in] opt OptimizerSetting object.
   * @param[in] planar_axis Optional planar axis.
   * @return Linearized dynamics factors.
   */
  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /**
   * Returns joint limit factors for the robot.
   *
   * @param[in] t    Timestep to return q factors for.
   * @param[in] opt  OptimizerSetting object.
   * @return joint limit factors.
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const;
};
}  // namespace gtdynamics
