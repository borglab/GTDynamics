/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file sdf.h
 * @brief Reading from SDF files.
 * @author Frank Dellaert, Alejandro Escontrela, Stephanie McCormick
 */

#pragma once

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include <string>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * Obtain the sdf ElementPtr associated with the robot model.
 *
 * @param sdf_file_path a string containing the absolute to the sdf file.
 * @param model_name name of the robot we care about. Must be specified in case
 * sdf_file_path points to a world file.
 * @return SDF Model
 */
sdf::Model GetSdf(std::string sdf_file_path, std::string model_name = "");

/**
 * @fn Construct a Link class from sdf::Link
 * @param[in] sdf_link 
 * @return LinkSharedPtr 
 */
LinkSharedPtr LinkFromSdf(const sdf::Link& sdf_link);

/**
 * @fn Construct a Joint class from sdf::Link
 * @param[in] parent_link  shared pointer to parent link
 * @param[in] chlid_link shared pointer to child link
 * @param[in] sdf_joint
 * @return LinkSharedPtr 
 */
JointSharedPtr JointFromSdf(const LinkSharedPtr& parent_link, const LinkSharedPtr& child_link, const sdf::Joint& sdf_joint);

/**
 * @fn Construct a Link from sdf file
 * @param[in] link_name  name of the specified link
 * @param[in] sdf_file_path path to sdf file
 * @param[in] model_name name of the robot
 * @return LinkSharedPtr 
 */
LinkSharedPtr LinkFromSdf(std::string link_name, std::string sdf_file_path, std::string model_name="");

/**
 * Parse a ignition::math Pose object into a gtsam::Pose.
 *
 * @param ignition_pose An ignition::math::Pose object to be parsed.
 */
gtsam::Pose3 Pose3FromIgnition(ignition::math::Pose3d ignition_pose);

/**
 * @fn Construct Robot from a urdf or sdf file.
 * @param[in] file_path path to the file.
 * @param[in] model_name name of the robot we care about. Must be specified in
 *    case sdf_file_path points to a world file.
 */
Robot CreateRobotFromFile(const std::string file_path,
                          std::string model_name = "");

/**
 * @fn Extract joint parameter values from an input sdf::Joint.
 * @param[in] sdf_joint a joint object which allows access to functions
 * needed to populate joint parameters.
 * @return a struct of parameters whose values have been set using
 * sdf::Joint functions.
 */
Joint::Parameters ParametersFromSdfJoint(const sdf::Joint &sdf_joint);

/**
 * @fn Extract joint parameter values from an input sdf::Link.
 * @param[in] sdf_link a link object which allows access to functions
 * needed to populate link parameters.
 * @return a struct of parameters whose values have been set using
 * sdf::Link functions.
 */
Link::Params ParametersFromSdfLink(const sdf::Link &sdf_link);

/**
 * @fn Get joint pose defined in world frame from an sdf::Joint object
 * @param[in] sdf_joint    a joint object which allows access to
 * functions needed to populate joint parameters.
 * @param[in] parent_link  Shared pointer to the parent Link.
 * @param[in] child_link   Shared pointer to the child Link.
 * @return Joint pose defined in world frame
 */
gtsam::Pose3 GetJointFrame(const sdf::Joint &sdf_joint,
                           const LinkSharedPtr &parent_link,
                           const LinkSharedPtr &child_link);

/**
 * @fn Converts an axis taken from input sdf::Joint into the Vector3 format
 * that GTSAM uses.
 * @param[in] sdf_joint a joint object which allows access to functions
 * needed to populate joint parameters.
 * @return a vector containing axis values extracted from SDF.
 */
gtsam::Vector3 GetSdfAxis(const sdf::Joint &sdf_joint);

}  // namespace gtdynamics
