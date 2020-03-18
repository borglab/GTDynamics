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

#include <sdf/parser_urdf.hh>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotTypes.h"

#include <boost/optional.hpp>


namespace gtdynamics {

/** Construct all Link and Joint objects from an input
 * urdf::ModelInterfaceSharedPtr. Keyword arguments: urdf_ptr         -- a
 * shared pointer to a urdf::ModelInterface object. joint_params     -- a vector
 * contanining optional params for joints.
 *
 */

typedef std::map<std::string, gtdynamics::LinkSharedPtr> LinkMap;
typedef std::map<std::string, gtdynamics::JointSharedPtr> JointMap;
typedef std::pair<LinkMap, JointMap> LinkJointPair;

/** Construct all Link and Joint objects from an input
 sdf::ElementPtr.
 * Keyword arguments:
 *    sdf_ptr          -- a shared pointer to a sdf::ElementPtr containing the
        robot model.
 *    joint_params     -- a vector contanining optional params for joints.
 *
 */
LinkJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<gtdynamics::JointParams>> joint_params =
        boost::none);

/** Construct all Link and Joint objects from an input urdf or sdf
 * file. Keyword arguments: file_path    -- absolute path to the urdf or sdf
 * file containing the robot description. joint_params -- a vector containing
 * optional params for joints.
 */
LinkJointPair extract_structure_from_file(
    const std::string file_path, const std::string model_name,
    const boost::optional<std::vector<gtdynamics::JointParams>> joint_params =
        boost::none);

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
   * Constructor.
   * Keyword Arguments:
   *  robot_links_and_joints    -- LinkJointPair containing links and joints.
   */
  explicit Robot(LinkJointPair links_and_joints);

  /** Constructor from a urdf or sdf file.
   *
   * Keyword Arguments:
   *  file_path -- path to the file.
   */
  explicit Robot(const std::string file_path, std::string model_name = "");

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
   * calculate forward kinematics by performing bfs in the link-joint graph
   * (will throw an error when invalid joint angle specification detected)
   * Keyword Arguments:
   *    joint_angles      -- joint angles for all joints
   *    joint_vels        -- joint velocities for all joints
   *    prior_link_name   -- name of link with known pose & twist
   *    prior_link_pose   -- pose of the known link
   *    prior_link_twist  -- twist of the konwn link
   * return poses and twists of all links
   */
  FKResults forwardKinematics(
      const JointValues &joint_angles, const JointValues &joint_vels,
      const boost::optional<std::string> prior_link_name = boost::none,
      const boost::optional<gtsam::Pose3> &prior_link_pose = boost::none,
      const boost::optional<gtsam::Vector6> &prior_link_twist =
          boost::none) const;

  /** Returns q factors for the robot.
   * 
   * Keyword Arguments:
   *    t               -- Timestep to return q factors for.
   *    opt             -- OptimizerSetting object.
   */
  gtsam::NonlinearFactorGraph qFactors(const int &t, const OptimizerSetting &opt) const;

  /** Returns v factors for the robot.
   * 
   * Keyword Arguments:
   *    t               -- Timestep to return q factors for.
   *    opt             -- OptimizerSetting object.
   */
  gtsam::NonlinearFactorGraph vFactors(const int &t, const OptimizerSetting &opt) const;

  /** Returns a factors for the robot.
   * 
   * Keyword Arguments:
   *    t               -- Timestep to return q factors for.
   *    opt             -- OptimizerSetting object.
   */
  gtsam::NonlinearFactorGraph aFactors(const int &t, const OptimizerSetting &opt) const;

  /** Returns dynamics factors for the robot.
   * 
   * Keyword Arguments:
   *    t               -- Timestep to return q factors for.
   *    opt             -- OptimizerSetting object.
   *    planar_axis     -- Optional planar axis.
   */
  gtsam::NonlinearFactorGraph dynamicsFactors(const int &t, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const;

  /** Returns joint limit factors for the robot.
   * 
   * Keyword Arguments:
   *    t               -- Timestep to return q factors for.
   *    opt             -- OptimizerSetting object.
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(const int &t, const OptimizerSetting &opt) const;

};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_ROBOT_H_
