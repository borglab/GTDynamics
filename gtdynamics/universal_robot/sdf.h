/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file sdf.h
 * @brief Reading from SDF files.
 * @Author: Frank Dellaert, Alejandro Escontrela, Stephanie McCormick
 */

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

#include <string>

namespace gtdynamics {

/** Construct Robot from a urdf or sdf file.
 * @param[in] file_path path to the file.
 * @param[in] model_name name of the robot we care about. Must be specified in
 *    case sdf_file_path points to a world file.
 */
Robot CreateRobotFromFile(const std::string file_path,
                          std::string model_name = "");

/** @fn Construct all Link and Joint objects from an input sdf::ElementPtr.
 * @param sdf_ptr a shared pointer to a sdf::ElementPtr containing the model.
 * @param joint_parameters a vector containing optional parameters for joints.
 * @return LinkMap and JointMap as a pair
 */
LinkJointPair ExtractRobotFromSdf(
    const sdf::Model sdf,
    const boost::optional<std::vector<ScrewJointBase::Parameters>>
        joint_parameters = boost::none);

/** @fn Extract joint parameter values from an input sdf::Joint.
 * @param[in] sdf_joint a joint object which allows access to functions
 * needed to populate joint parameters.
 * @return a struct of parameters whose values have been set using
 * sdf::Joint functions.
 */
ScrewJointBase::Parameters ParametersFromFile(
    const sdf::Joint &sdf_joint);

/** @fn Get joint pose defined in world frame from an sdf::Joint object
 * @param[in] sdf_joint    a joint object which allows access to
 * functions needed to populate joint parameters.
 * @param[in] parent_link  Shared pointer to the parent Link.
 * @param[in] child_link   Shared pointer to the child Link.
 * @return Joint pose defined in world frame 
 */
gtsam::Pose3 GetJointFrame(const sdf::Joint &sdf_joint,
                           const LinkSharedPtr &parent_link,
                           const LinkSharedPtr &child_link);

/** @fn Converts an axis taken from input sdf::Joint into the Vector3 format
 * that GTSAM uses.
 * @param[in] sdf_joint a joint object which allows access to functions
 * needed to populate joint parameters.
 * @return a vector containing axis values extracted from SDF.
 */
gtsam::Vector3 GetSdfAxis(const sdf::Joint &sdf_joint);

} // namespace gtdynamics
