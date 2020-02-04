/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RobotModels.h
 * @brief robot models for tests
 * @Author: Yetong Zhang and Alejandro Escontrela
 */

#pragma once

#include <DynamicsGraph.h>
#include <UniversalRobot.h>

#include <string>

// using namespace std;
// using namespace robot;
// using namespace gtsam;
using robot::UniversalRobot;

namespace four_bar_linkage {
UniversalRobot getFourBar() {
  UniversalRobot four_bar =
      UniversalRobot(std::string(SDF_PATH) + "/test/four_bar_linkage_pure.sdf");
  return four_bar;
}
// Load the robot from urdf file
UniversalRobot my_robot = getFourBar();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace four_bar_linkage

namespace simple_urdf {
UniversalRobot getSimpleUrdf() {
  UniversalRobot simple_robot =
      UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  simple_robot.getLinkByName("l1")->fix();
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf

namespace simple_urdf_zero_inertia {
UniversalRobot getSimpleUrdf() {
  UniversalRobot simple_robot = UniversalRobot(
      std::string(URDF_PATH) + "/test/simple_urdf_zero_inertia.urdf");
  simple_robot.getLinkByName("l1")->fix();
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf_zero_inertia

namespace simple_urdf_eq_mass {
UniversalRobot getSimpleUrdfEqMass() {
  UniversalRobot simple_robot =
      UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf_eq_mass.urdf");
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdfEqMass();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
}  // namespace simple_urdf_eq_mass

namespace jumping_robot
{
UniversalRobot getJumpingRobot()
{
  UniversalRobot jumping_robot = 
      UniversalRobot(std::string(SDF_PATH) + "/test/jumping_robot.sdf");
  jumping_robot.getLinkByName("l0")->fix();
  return jumping_robot;
}
// Load the robot from urdf file
UniversalRobot my_robot = getJumpingRobot();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();
gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
} // namespace jumping_robot