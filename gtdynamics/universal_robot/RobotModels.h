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

namespace four_bar_linkage {
gtdynamics::Robot getFourBar() {
  gtdynamics::Robot four_bar = gtdynamics::CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/four_bar_linkage_pure.sdf");
  return four_bar;
}
// Load the robot from urdf file
gtdynamics::Robot robot = getFourBar();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace four_bar_linkage

namespace simple_urdf {
gtdynamics::Robot getSimpleUrdf() {
  auto robot = gtdynamics::CreateRobotFromFile(std::string(URDF_PATH) +
                                               "/test/simple_urdf.urdf");
  robot.fixLink("l1");
  return robot;
}
gtdynamics::Robot robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf

namespace simple_urdf_prismatic {
gtdynamics::Robot getSimpleUrdf() {
  auto robot = gtdynamics::CreateRobotFromFile(
      std::string(URDF_PATH) + "/test/simple_urdf_prismatic.urdf");
  robot.fixLink("l1");
  return robot;
}
gtdynamics::Robot robot = getSimpleUrdf();
}  // namespace simple_urdf_prismatic

namespace simple_urdf_zero_inertia {
gtdynamics::Robot getSimpleUrdf() {
  auto robot = gtdynamics::CreateRobotFromFile(
      std::string(URDF_PATH) + "/test/simple_urdf_zero_inertia.urdf");
  robot.fixLink("l1");
  return robot;
}
gtdynamics::Robot robot = getSimpleUrdf();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf_zero_inertia

namespace simple_urdf_eq_mass {
gtdynamics::Robot getSimpleUrdfEqMass() {
  auto robot = gtdynamics::CreateRobotFromFile(
      std::string(URDF_PATH) + "/test/simple_urdf_eq_mass.urdf");
  return robot;
}
gtdynamics::Robot robot = getSimpleUrdfEqMass();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_urdf_eq_mass

namespace simple_rr {
gtdynamics::Robot getSimpleRR() {
  auto robot = gtdynamics::CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");
  return robot;
}
gtdynamics::Robot robot = getSimpleRR();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace simple_rr

namespace jumping_robot {
gtdynamics::Robot getJumpingRobot() {
  gtdynamics::Robot jumping_robot = gtdynamics::CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/jumping_robot.sdf");
  jumping_robot.fixLink("l0");
  return jumping_robot;
}
// Load the robot from urdf file
gtdynamics::Robot robot = getJumpingRobot();
gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

}  // namespace jumping_robot

/* *************************
 *  Walking Robots         *
 ************************* */
namespace Atlas {
/// Bipedal leg keys
inline DynamicsSymbol Right(uint64_t j) { return internal::PoseKey(0, j); }
inline DynamicsSymbol Left(uint64_t j) { return internal::PoseKey(1, j); }
}  // namespace Atlas

namespace A1 {
/**
 * Quadruped leg keys
 * The IDs are the link IDs for the robot.
 */
inline DynamicsSymbol FR(uint64_t j) { return internal::PoseKey(6, j); }
inline DynamicsSymbol FL(uint64_t j) { return internal::PoseKey(3, j); }
inline DynamicsSymbol RR(uint64_t j) { return internal::PoseKey(12, j); }
inline DynamicsSymbol RL(uint64_t j) { return internal::PoseKey(9, j); }
}  // namespace A1
