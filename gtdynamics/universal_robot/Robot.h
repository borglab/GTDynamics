/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Robot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_ROBOT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_ROBOT_H_

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

#include <boost/optional.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace gtdynamics {

typedef std::map<std::string, LinkSharedPtr> LinkMap;
typedef std::map<std::string, JointSharedPtr> JointMap;
typedef std::pair<LinkMap, JointMap> LinkJointPair;

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

  /** Constructor from link and joint elements.
   *  @param[in] robot_links_and_joints LinkJointPair containing links
   *    and joints.
   */
  explicit Robot(LinkJointPair links_and_joints);

  /// Return this robot's links.
  std::vector<LinkSharedPtr> links() const;

  /// Return this robot's joints.
  std::vector<JointSharedPtr> joints() const;

  /// remove specified link from the robot
  void removeLink(LinkSharedPtr link);

  /// remove specified joint from the robot
  void removeJoint(JointSharedPtr joint);

  /// Return the link corresponding to the input string.
  LinkSharedPtr getLinkByName(std::string name) const;

  /// Return the joint corresponding to the input string.
  JointSharedPtr getJointByName(std::string name) const;

  /// Return number of *moving* links.
  int numLinks() const;

  /// Return number of joints.
  int numJoints() const;

  // print links and joints of the robot, for debug purposes
  void printRobot() const;

  // map from joint name to joint angle/vel/accel/torque
  typedef std::map<std::string, double> JointValues;

  // map from link name to link pose
  typedef std::map<std::string, gtsam::Pose3> LinkPoses;

  // map from link name to link twist
  typedef std::map<std::string, gtsam::Vector6> LinkTwists;

  // type for storing forward kinematics results
  typedef std::pair<LinkPoses, LinkTwists> FKResults;

  /**
   * @fn calculate forward kinematics by performing bfs in the link-joint graph
   * (will throw an error when invalid joint angle specification detected)
   *
   * @param[in] joint_angles     joint angles for all joints
   * @param[in] joint_vels       joint velocities for all joints
   * @param[in] prior_link_name  name of link with known pose & twist
   * @param[in] prior_link_pose  pose of the known link
   * @param[in] prior_link_twist twist of the konwn link
   * @return poses and twists of all links
   */
  FKResults forwardKinematics(
      const JointValues &joint_angles, const JointValues &joint_vels,
      const boost::optional<std::string> prior_link_name = boost::none,
      const boost::optional<gtsam::Pose3> &prior_link_pose = boost::none,
      const boost::optional<gtsam::Vector6> &prior_link_twist =
          boost::none) const;

  /** @fn Returns pose factors for the robot.
   *
   * @param[in] t    Timestep to return q factors for.
   * @param[in] opt  OptimizerSetting object.
   * @return pose factors.
   */
  gtsam::NonlinearFactorGraph qFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /** @fn Returns velocity factors for the robot.
   *
   * @param[in]t    Timestep to return q factors for.
   * @param[in]opt  OptimizerSetting object.
   * @return velocity factors.
   */
  gtsam::NonlinearFactorGraph vFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /** @fn Returns acceleration factors for the robot.
   *
   * @param[in] t    Timestep to return q factors for.
   * @param[in] opt  OptimizerSetting object.
   * @return acceleration factors.
   */
  gtsam::NonlinearFactorGraph aFactors(size_t t,
                                       const OptimizerSetting &opt) const;

  /** @fn Returns linear forward dynamics priors for the robot.
   *
   * @param[in] t             Timestep to return q factors for.
   * @param[in] torques       Joint torques.
   * @param[in] opt           OptimizerSetting object.
   * @return Linear forward dynamics priors.
   */
  gtsam::GaussianFactorGraph linearFDPriors(size_t t,
                                            const JointValues &torques,
                                            const OptimizerSetting &opt) const;

  /** @fn Returns accel factors linearized about specified operating
   *    condition.
   *
   * @param[in] t             Timestep to return q factors for.
   * @param[in] poses         Link poses.
   * @param[in] twists        Link twists.
   * @param[in] joint_angles  Joint angles.
   * @param[in] joint_vels    Joint velocities.
   * @param[in] opt           OptimizerSetting object.
   * @param[in] planar_axis    Optional planar axis.
   * @return Linearized accel factors.
   */
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /** @fn Returns dynamics factors for the robot.
   *
   * @param[in] t          Timestep to return q factors for.
   * @param[in] opt        OptimizerSetting object.
   * @param[in]planar_axis Optional planar axis.
   * @return dynamics factors.
   */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const;

  /** @fn Returns dynamics factors linearized about specified operating
   *    condition.
   *
   * @param[in] t             Timestep to return q factors for.
   * @param[in] poses         Link poses.
   * @param[in] twists        Link twists.
   * @param[in] joint_angles  Joint angles.
   * @param[in] joint_vels    Joint velocities.
   * @param[in] opt           OptimizerSetting object.
   * @param[in] planar_axis    Optional planar axis.
   * @return Linearized dynamics factors.
   */
  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /** @fn Returns joint limit factors for the robot.
   *
   * @param[in] t    Timestep to return q factors for.
   * @param[in] opt  OptimizerSetting object.
   * @return joint limit factors.
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const;
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_ROBOT_H_
