/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotModels.h
 * @brief robot models for tests
 * @author Yetong Zhang and Alejandro Escontrela
 */

#pragma once

#include <string>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

// TODO(aescontrela): The entire program shouldn't crash when a single file
// doesn't load.

using namespace gtdynamics;

namespace four_bar_linkage {
Robot getFourBar() {
  Robot four_bar = CreateRobotFromFile(std::string(SDF_PATH) +
                                       "/test/four_bar_linkage_pure.sdf");
  return four_bar;
}
// Load the robot from urdf file
Robot my_robot = getFourBar();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace four_bar_linkage

namespace simple_urdf {
Robot getSimpleUrdf() {
  Robot simple_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  simple_robot.fixLink("l1");
  return simple_robot;
}
Robot my_robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf

namespace simple_urdf_zero_inertia {
Robot getSimpleUrdf() {
  Robot simple_robot = CreateRobotFromFile(
      std::string(URDF_PATH) + "/test/simple_urdf_zero_inertia.urdf");
  simple_robot.fixLink("l1");
  return simple_robot;
}
Robot my_robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf_zero_inertia

namespace simple_urdf_eq_mass {
Robot getSimpleUrdfEqMass() {
  Robot simple_robot = CreateRobotFromFile(std::string(URDF_PATH) +
                                           "/test/simple_urdf_eq_mass.urdf");
  return simple_robot;
}
Robot my_robot = getSimpleUrdfEqMass();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf_eq_mass

namespace simple_rr {
Robot getSimpleRR() {
  Robot simple_robot = CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");
  return simple_robot;
}
Robot my_robot = getSimpleRR();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_rr

namespace jumping_robot {
Robot getJumpingRobot() {
  Robot jumping_robot =
      CreateRobotFromFile(std::string(SDF_PATH) + "/test/jumping_robot.sdf");
  jumping_robot.fixLink("l0");
  return jumping_robot;
}
// Load the robot from urdf file
Robot my_robot = getJumpingRobot();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace jumping_robot
