/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testUniversalRobotChanges.cpp
 * @brief Test new features in Universal Robot classes.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

TEST(Link, RenameReassign) {
  Link l1(1, "l1", 100.0, gtsam::Vector3(3, 2, 1).asDiagonal(),
          Pose3(Rot3(), Point3(0, 0, 1)), Pose3());

  // Test Rename
  l1.rename("link1");
  EXPECT(assert_equal("link1", l1.name()));

  // Test Reassign
  l1.reassign(2);
  EXPECT_LONGS_EQUAL(2, l1.id());
}

TEST(Joint, RenameReassign) {
  auto robot = simple_urdf::getRobot();
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");
  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0.5, 2)), l1, l2,
                   gtsam::Vector3(1, 0, 0), JointParams());

  // Test Rename
  j1.rename("joint1");
  EXPECT(assert_equal("joint1", j1.name()));

  // Test Reassign
  j1.reassign(2);
  EXPECT_LONGS_EQUAL(2, j1.id());
}

TEST(Robot, RenameReassign) {
  Robot robot = simple_urdf::getRobot();

  // Test Rename Links
  std::map<std::string, std::string> link_name_map = {{"l1", "link1"},
                                                      {"l2", "link2"}};
  robot.renameLinks(link_name_map);
  EXPECT(robot.link("link1"));
  EXPECT(robot.link("link2"));
  THROWS_EXCEPTION(robot.link("l1"));
  THROWS_EXCEPTION(robot.link("l2"));

  // Test Rename Joints
  std::map<std::string, std::string> joint_name_map = {{"j1", "joint1"}};
  robot.renameJoints(joint_name_map);
  EXPECT(robot.joint("joint1"));
  THROWS_EXCEPTION(robot.joint("j1"));

  // Test Reassign Links
  std::vector<std::string> ordered_link_names = {"link2", "link1"};
  robot.reassignLinks(ordered_link_names);
  EXPECT_LONGS_EQUAL(0, robot.link("link2")->id());
  EXPECT_LONGS_EQUAL(1, robot.link("link1")->id());

  // Test Reassign Joints
  std::vector<std::string> ordered_joint_names = {"joint1"};
  robot.reassignJoints(ordered_joint_names);
  EXPECT_LONGS_EQUAL(0, robot.joint("joint1")->id());

  // Test Ordered Links
  auto links = robot.orderedLinks();
  EXPECT_LONGS_EQUAL(2, links.size());
  EXPECT(assert_equal("link2", links[0]->name()));
  EXPECT(assert_equal("link1", links[1]->name()));

  // Test Ordered Joints
  auto joints = robot.orderedJoints();
  EXPECT_LONGS_EQUAL(1, joints.size());
  EXPECT(assert_equal("joint1", joints[0]->name()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
