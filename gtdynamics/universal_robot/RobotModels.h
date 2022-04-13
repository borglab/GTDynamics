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

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>

#include <string>

using gtdynamics::kSdfPath;
using gtdynamics::kUrdfPath;

namespace four_bar_linkage_pure {
gtdynamics::Robot getRobot() {
  gtdynamics::Robot four_bar = gtdynamics::CreateRobotFromFile(
      kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));
  return four_bar;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace four_bar_linkage_pure

namespace simple_urdf {
gtdynamics::Robot getRobot() {
  auto robot = gtdynamics::CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf.urdf"));
  robot = robot.fixLink("l1");
  return robot;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf

namespace simple_urdf_prismatic {
gtdynamics::Robot getRobot() {
  auto robot = gtdynamics::CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));
  robot = robot.fixLink("l1");
  return robot;
}
}  // namespace simple_urdf_prismatic

namespace simple_urdf_zero_inertia {
gtdynamics::Robot getRobot() {
  auto robot = gtdynamics::CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_zero_inertia.urdf"));
  robot = robot.fixLink("l1");
  return robot;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf_zero_inertia

namespace simple_urdf_eq_mass {
gtdynamics::Robot getRobot() {
  auto robot = gtdynamics::CreateRobotFromFile(
      kUrdfPath + std::string("test/simple_urdf_eq_mass.urdf"));
  return robot;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf_eq_mass

namespace simple_rr {
gtdynamics::Robot getRobot() {
  auto robot = gtdynamics::CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");
  return robot;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_rr

namespace jumping_robot {
gtdynamics::Robot getRobot() {
  gtdynamics::Robot jumping_robot = gtdynamics::CreateRobotFromFile(
      kSdfPath + std::string("test/jumping_robot.sdf"));
  jumping_robot = jumping_robot.fixLink("l0");
  return jumping_robot;
}
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace jumping_robot
