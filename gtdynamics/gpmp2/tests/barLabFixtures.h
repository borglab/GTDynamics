/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  barLabFixtures.h
 * @brief Shared bar_lab robot fixtures and configurations for the gpmp2 tests.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/config.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <string>
#include <vector>

namespace gtdynamics {

/// Obstacle grid geometry shared by the SDF-based tests.
inline constexpr double kCell = 0.05;
inline constexpr double kRadius = 0.15;
inline constexpr double kEpsilon = 0.10;

/// Half-cell grid offset: the trilinear gradient is discontinuous on nodes.
inline constexpr double kHalfCell = 0.5 * kCell;

/// The bar_lab workspace, loaded once per test binary.
inline const Robot &barLabRobot() {
  static const Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));
  return robot;
}

/// Look up joints on the bar_lab robot by name.
inline std::vector<JointSharedPtr> jointsByName(
    const std::vector<std::string> &names) {
  std::vector<JointSharedPtr> joints;
  for (auto &&name : names) joints.push_back(barLabRobot().joint(name));
  return joints;
}

/// The nine movable joints of robot1: gantry prismatic then arm revolute.
inline std::vector<JointSharedPtr> robot1Joints() {
  return jointsByName({"bridge1_joint_EA_X", "robot1_joint_EA_Y",
                       "robot1_joint_EA_Z", "robot1_joint_1", "robot1_joint_2",
                       "robot1_joint_3", "robot1_joint_4", "robot1_joint_5",
                       "robot1_joint_6"});
}

/// All eighteen joints, robot1's nine then robot2's nine.
inline std::vector<JointSharedPtr> bothArmJoints() {
  return jointsByName(
      {"bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
       "robot1_joint_1", "robot1_joint_2", "robot1_joint_3", "robot1_joint_4",
       "robot1_joint_5", "robot1_joint_6", "bridge2_joint_EA_X",
       "robot2_joint_EA_Y", "robot2_joint_EA_Z", "robot2_joint_1",
       "robot2_joint_2", "robot2_joint_3", "robot2_joint_4", "robot2_joint_5",
       "robot2_joint_6"});
}

/// Two query points on robot1's wrist: the link CoM and a point out along the
/// tool.
inline std::vector<PointOnLink> wristPoints() {
  const LinkSharedPtr link = barLabRobot().link("robot1_link_6");
  return {PointOnLink(link, gtsam::Point3(0.0, 0.0, 0.0)),
          PointOnLink(link, gtsam::Point3(0.0, 0.0, 0.1))};
}

/// The two wrists, one query point each, for cross-arm pairs.
inline std::vector<PointOnLink> crossArmWristPoints() {
  return {
      PointOnLink(barLabRobot().link("robot1_link_6"), gtsam::Point3(0, 0, 0)),
      PointOnLink(barLabRobot().link("robot2_link_6"), gtsam::Point3(0, 0, 0))};
}

/// Robot1 start and goal, one bridge metre apart.
inline gtsam::Vector startConfig() {
  return (gtsam::Vector(9) << 2.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0)
      .finished();
}
inline gtsam::Vector goalConfig() {
  return (gtsam::Vector(9) << 3.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0)
      .finished();
}

/// Arm pose used by every both-arm configuration.
inline const gtsam::Vector kArm =
    (gtsam::Vector(6) << 0.2, -0.5, -1.0, 0.3, 0.5, 0.2).finished();

/// Bridges far apart on the rail.
inline gtsam::Vector bothArmsApart() {
  gtsam::Vector q(18);
  q << 2.0, 3.0, 1.0, kArm, 9.0, 3.0, 1.0, kArm;
  return q;
}

/// Bridges 0.2 m apart, so the two arm bases nearly coincide.
inline gtsam::Vector bothArmsClose() {
  gtsam::Vector q(18);
  q << 5.0, 3.0, 1.0, kArm, 5.2, 3.0, 1.0, kArm;
  return q;
}

/// The bridges have moved a metre toward each other from bothArmsApart.
inline gtsam::Vector bothArmsStepCloser() {
  gtsam::Vector q(18);
  q << 3.0, 3.0, 1.0, kArm, 8.0, 3.0, 1.0, kArm;
  return q;
}

/// Constant velocity taking bothArmsApart to bothArmsStepCloser over 0.5 s.
inline gtsam::Vector bothArmsVelocity() {
  gtsam::Vector v = gtsam::Vector::Zero(18);
  v(0) = 2.0;
  v(9) = -2.0;
  return v;
}

}  // namespace gtdynamics
