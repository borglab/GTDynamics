/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file sdf.h
 * @brief Reading from SDF files, exposed functions.
 * @author Frank Dellaert, Alejandro Escontrela, Stephanie McCormick
 */

#pragma once

#include <string>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @fn Construct Robot from a urdf or sdf file.
 * @param[in] file_path path to the file.
 * @param[in] model_name name of the robot we care about. Must be specified in
 *    case sdf_file_path points to a world file.
 */
Robot CreateRobotFromFile(const std::string &file_path,
                          const std::string &model_name = "");

}  // namespace gtdynamics
