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

/** @fn Construct all Link and Joint objects from an input sdf::ElementPtr.
 * @param sdf_ptr a shared pointer to a sdf::ElementPtr containing the robot
 * model.
 * @param joint_parameters a vector contanining optional parameters for joints.
 * @return LinkMap and JointMap as a pair
 */
// LinkJointPair ExtractRobotFromSdf(
//     const sdf::Model sdf,
//     const boost::optional<std::vector<ScrewJointBase::Parameters>>
//         joint_parameters = boost::none);

/** @fn Construct all Link and Joint objects from an input urdf or sdf file.
 * @param[in] file_path absolute path to the urdf or sdf file containing the
 * robot description.
 * @param[in] model_name name of the robot we care about. Must be specified in
 * case sdf_file_path points to a world file.
 * @param[in] joint_parameters a vector contanining optional parameters for
 * joints.
 * @return LinkMap and JointMap as a pair
 */
// LinkJointPair ExtractRobotFromFile(
//     const std::string file_path, const std::string model_name,
//     const boost::optional<std::vector<ScrewJointBase::Parameters>>
//         joint_parameters = boost::none);

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

ScrewJointBase::Parameters ParametersFromSDF(
    const sdf::Joint &sdf_joint);

gtsam::Pose3 GetJointFrame(const sdf::Joint &sdf_joint,
                           const LinkSharedPtr &parent_link,
                           const LinkSharedPtr &child_link);

gtsam::Vector3 GetSdfAxis(const sdf::Joint &sdf_joint);

}
