/**
 * @file UniversalRobot.h
 * @brief Robot structure.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <RobotJoint.h>
#include <RobotLink.h>
#include <RobotTypes.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <utils.h>

#include <boost/optional.hpp>
#include <sdf/parser_urdf.hh>
#include <sstream>
#include <stdexcept>
#include <vector>

// TODO(aescontrela): Add `const` to instance methods that don't modify the
// object's data members.

namespace robot {

/** Construct all RobotLink and RobotJoint objects from an input
 * urdf::ModelInterfaceSharedPtr. Keyword arguments: urdf_ptr         -- a
 * shared pointer to a urdf::ModelInterface object. joint_params     -- a vector
 * contanining optional params for joints.
 *
 */
typedef std::pair<std::vector<robot::RobotLinkSharedPtr>,
                  std::vector<robot::RobotJointSharedPtr>>
    RobotJointPair;

/** Construct all RobotLink and RobotJoint objects from an input
 sdf::ElementPtr.
 * Keyword arguments:
 *    sdf_ptr          -- a shared pointer to a sdf::ElementPtr containing the
        robot model.
 *    joint_params     -- a vector contanining optional params for joints.
 *
 */
RobotJointPair extract_structure_from_sdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params =
        boost::none);

/** Construct all RobotLink and RobotJoint objects from an input urdf or sdf
 * file. Keyword arguments: file_path    -- absolute path to the urdf or sdf
 * file containing the robot description. joint_params -- a vector containing
 * optional params for joints.
 */
RobotJointPair extract_structure_from_file(
    const std::string file_path, const std::string model_name,
    const boost::optional<std::vector<robot::RobotJointParams>> joint_params =
        boost::none);

class UniversalRobot {
 private:
  std::vector<RobotLinkSharedPtr> link_bodies_;
  std::vector<RobotJointSharedPtr> link_joints_;

  // For quicker/easier access to links and joints.
  std::map<std::string, robot::RobotLinkSharedPtr> name_to_link_body_;
  std::map<std::string, robot::RobotJointSharedPtr> name_to_link_joint_;

 public:
  /**
   * Construct a robot structure using a URDF model interface.
   * Keyword Arguments:
   *  robot_links_and_joints    -- RobotJointPair containing links and joints.
   *
   */
  UniversalRobot(RobotJointPair links_and_joints);

  /** Construct a robot structure directly from a urdf or sdf file.
   *
   * Keyword Arguments:
   *  file_path -- path to the file.
   */
  UniversalRobot(const std::string file_path, std::string model_name = "");

  /// Return this robot's links.
  std::vector<RobotLinkSharedPtr> links() const;

  /// Return this robot's joints.
  std::vector<RobotJointSharedPtr> joints() const;

  /// remove specified link from the robot
  void removeLink(RobotLinkSharedPtr link);

  /// remove specified joint from the robot
  void removeJoint(RobotJointSharedPtr joint);

  /// Return the link corresponding to the input string.
  RobotLinkSharedPtr getLinkByName(std::string name);

  /// Return the joint corresponding to the input string.
  RobotJointSharedPtr getJointByName(std::string name);

  /// Return number of *moving* links.
  int numLinks() const;

  /// Return number of joints.
  int numJoints() const;

  // print links and joints of the robot, for debug purposes
  void printRobot() const;
};
}  // namespace robot
