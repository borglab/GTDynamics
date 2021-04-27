/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  contactGoalsExample.h
 * @brief Examples for contact goals for Vision60 robot.
 * @author: Frank Dellaert, Yetong Zhang
 */

#pragma once

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/sdf.h>

namespace gtdynamics {
namespace contact_goals_example {
const Robot robot =
    CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"));

const gtsam::Point3 contact_in_com(0.14, 0, 0);
const LinkSharedPtr LH = robot.link("lower1"), LF = robot.link("lower0"),
                    RF = robot.link("lower2"), RH = robot.link("lower3");
const ContactGoals contact_goals = {
    {{LH, contact_in_com}, {-0.4, 0.16, -0.2}},
    {{LF, contact_in_com}, {0.3, 0.16, -0.2}},
    {{RF, contact_in_com}, {0.3, -0.16, -0.2}},
    {{RH, contact_in_com}, {-0.4, -0.16, -0.2}}};

}  // namespace contact_goals_example
}  // namespace gtdynamics
