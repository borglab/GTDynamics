/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testRobot.cpp
 * @brief Test Robot instance methods and integration test with various
 * URDF/SDF configurations.
 * @author Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal;

TEST(Robot, four_bar_sdf) {
  // Initialize Robot instance from a file.
  Robot four_bar =
      CreateRobotFromFile(std::string(SDF_PATH) + "/test/four_bar_linkage.sdf");

  // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(5, four_bar.links().size()));
  EXPECT(assert_equal(5, four_bar.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("l0", four_bar.getLinkByName("l0")->name()));
  EXPECT(assert_equal("l1", four_bar.getLinkByName("l1")->name()));
  EXPECT(assert_equal("l2", four_bar.getLinkByName("l2")->name()));
  EXPECT(assert_equal("l3", four_bar.getLinkByName("l3")->name()));
  EXPECT(assert_equal("l4", four_bar.getLinkByName("l4")->name()));
  EXPECT(assert_equal("j0", four_bar.getJointByName("j0")->name()));
  EXPECT(assert_equal("j1", four_bar.getJointByName("j1")->name()));
  EXPECT(assert_equal("j2", four_bar.getJointByName("j2")->name()));
  EXPECT(assert_equal("j3", four_bar.getJointByName("j3")->name()));
  EXPECT(assert_equal("j4", four_bar.getJointByName("j4")->name()));
}

TEST(Robot, simple_rr_sdf) {
  // Initialize Robot instance from a file.
  Robot simple_rr = CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  // // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(3, simple_rr.links().size()));
  EXPECT(assert_equal(2, simple_rr.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("link_0", simple_rr.getLinkByName("link_0")->name()));
  EXPECT(assert_equal("link_1", simple_rr.getLinkByName("link_1")->name()));
  EXPECT(assert_equal("link_2", simple_rr.getLinkByName("link_2")->name()));

  EXPECT(assert_equal("joint_1", simple_rr.getJointByName("joint_1")->name()));
  EXPECT(assert_equal("joint_2", simple_rr.getJointByName("joint_2")->name()));
}

TEST(Robot, removeLink) {
  // Initialize Robot instance from a file.
  Robot four_bar = CreateRobotFromFile(std::string(SDF_PATH) +
                                       "/test/four_bar_linkage_pure.sdf");
  four_bar.removeLink(four_bar.getLinkByName("l2"));
  EXPECT(four_bar.numLinks() == 3);
  EXPECT(four_bar.numJoints() == 2);
  EXPECT(four_bar.getLinkByName("l1")->getJoints().size() == 1);
  EXPECT(four_bar.getLinkByName("l3")->getJoints().size() == 1);
}

TEST(Robot, forwardKinematics) {
  Robot simple_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  JointValues joint_angles, joint_vels;
  joint_angles["j1"] = 0;
  joint_vels["j1"] = 0;

  // not fixing a link would cause an exception
  THROWS_EXCEPTION(simple_robot.forwardKinematics(joint_angles, joint_vels));

  // test fk at rest
  simple_robot.getLinkByName("l1")->fix();
  FKResults fk_results =
      simple_robot.forwardKinematics(joint_angles, joint_vels);
  LinkPoses poses = fk_results.first;
  LinkTwists twists = fk_results.second;

  gtsam::Pose3 T_wl1_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 1));
  gtsam::Pose3 T_wl2_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 3));
  gtsam::Vector6 V_l1_rest, V_l2_rest;
  V_l1_rest << 0, 0, 0, 0, 0, 0;
  V_l2_rest << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(T_wl1_rest, poses.at("l1")));
  EXPECT(assert_equal(T_wl2_rest, poses.at("l2")));
  EXPECT(assert_equal(V_l1_rest, twists.at("l1")));
  EXPECT(assert_equal(V_l2_rest, twists.at("l2")));

  // test fk with moving joint and fixed base
  joint_angles["j1"] = M_PI_2;
  joint_vels["j1"] = 1;

  fk_results = simple_robot.forwardKinematics(joint_angles, joint_vels);
  poses = fk_results.first;
  twists = fk_results.second;

  gtsam::Pose3 T_wl1_move(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 1));
  gtsam::Pose3 T_wl2_move(gtsam::Rot3::Rx(M_PI_2), gtsam::Point3(0, -1, 2));
  gtsam::Vector6 V_l1_move, V_l2_move;
  V_l1_move << 0, 0, 0, 0, 0, 0;
  V_l2_move << 1, 0, 0, 0, -1, 0;
  EXPECT(assert_equal(T_wl1_move, poses.at("l1")));
  EXPECT(assert_equal(T_wl2_move, poses.at("l2")));
  EXPECT(assert_equal(V_l1_move, twists.at("l1")));
  EXPECT(assert_equal(V_l2_move, twists.at("l2")));

  // test fk with moving joint and moving base
  simple_robot.getLinkByName("l1")->unfix();
  gtsam::Pose3 T_wl1_float(gtsam::Rot3::Rx(-M_PI_2), gtsam::Point3(0, 1, 1));
  gtsam::Pose3 T_wl2_float(gtsam::Rot3::Rx(0), gtsam::Point3(0, 2, 2));
  gtsam::Vector6 V_l1_float, V_l2_float;
  V_l1_float << 1, 0, 0, 0, -1, 0;
  V_l2_float << 2, 0, 0, 0, -2, 2;

  std::string prior_link_name = "l1";
  fk_results = simple_robot.forwardKinematics(
      joint_angles, joint_vels, prior_link_name, T_wl1_float, V_l1_float);
  poses = fk_results.first;
  twists = fk_results.second;
  EXPECT(assert_equal(T_wl1_float, poses.at("l1")));
  EXPECT(assert_equal(T_wl2_float, poses.at("l2")));
  EXPECT(assert_equal(V_l1_float, twists.at("l1")));
  EXPECT(assert_equal(V_l2_float, twists.at("l2")));
}

TEST(Robot, forwardKinematics_rpr) {
  Robot rpr_robot = CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rpr.sdf", "simple_rpr_sdf");

  JointValues joint_angles, joint_vels;
  joint_angles["joint_1"] = 0;
  joint_vels["joint_1"] = 0;
  joint_angles["joint_2"] = 0;
  joint_vels["joint_2"] = 0;
  joint_angles["joint_3"] = 0;
  joint_vels["joint_3"] = 0;

  // test fk at rest
  rpr_robot.getLinkByName("link_0")->fix();
  FKResults fk_results = rpr_robot.forwardKinematics(joint_angles, joint_vels);
  LinkPoses poses = fk_results.first;
  LinkTwists twists = fk_results.second;

  gtsam::Pose3 T_wl0_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1));
  gtsam::Pose3 T_wl1_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.5));
  gtsam::Pose3 T_wl2_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 1.1));
  gtsam::Pose3 T_wl3_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 1.7));
  gtsam::Vector6 V_l0_rest, V_l1_rest, V_l2_rest, V_l3_rest;
  V_l0_rest << 0, 0, 0, 0, 0, 0;
  V_l1_rest << 0, 0, 0, 0, 0, 0;
  V_l2_rest << 0, 0, 0, 0, 0, 0;
  V_l3_rest << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(T_wl0_rest, poses.at("link_0")));
  EXPECT(assert_equal(T_wl1_rest, poses.at("link_1")));
  EXPECT(assert_equal(T_wl2_rest, poses.at("link_2")));
  EXPECT(assert_equal(T_wl3_rest, poses.at("link_3")));
  EXPECT(assert_equal(V_l0_rest, twists.at("link_0")));
  EXPECT(assert_equal(V_l1_rest, twists.at("link_1")));
  EXPECT(assert_equal(V_l2_rest, twists.at("link_2")));
  EXPECT(assert_equal(V_l3_rest, twists.at("link_3")));

  // test fk with moving prismatic joint and fixed base
  joint_angles["joint_1"] = M_PI_2;
  joint_angles["joint_2"] = 0.5;
  joint_vels["joint_2"] = 1;

  fk_results = rpr_robot.forwardKinematics(joint_angles, joint_vels);
  poses = fk_results.first;
  twists = fk_results.second;

  gtsam::Pose3 T_wl0_move(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1));
  gtsam::Pose3 T_wl1_move(gtsam::Rot3::Ry(M_PI_2), gtsam::Point3(0.3, 0, 0.2));
  gtsam::Pose3 T_wl2_move(gtsam::Rot3::Ry(M_PI_2), gtsam::Point3(1.4, 0, 0.2));
  gtsam::Pose3 T_wl3_move(gtsam::Rot3::Ry(M_PI_2), gtsam::Point3(2.0, 0, 0.2));
  gtsam::Vector6 V_l0_move, V_l1_move, V_l2_move, V_l3_move;
  V_l0_move << 0, 0, 0, 0, 0, 0;
  V_l1_move << 0, 0, 0, 0, 0, 0;
  V_l2_move << 0, 0, 0, 0, 0, 1;
  V_l3_move << 0, 0, 0, 0, 0, 1;
  EXPECT(assert_equal(T_wl0_move, poses.at("link_0")));
  EXPECT(assert_equal(T_wl1_move, poses.at("link_1")));
  EXPECT(assert_equal(T_wl2_move, poses.at("link_2")));
  EXPECT(assert_equal(T_wl3_move, poses.at("link_3")));
  EXPECT(assert_equal(V_l0_move, twists.at("link_0")));
  EXPECT(assert_equal(V_l1_move, twists.at("link_1")));
  EXPECT(assert_equal(V_l2_move, twists.at("link_2")));
  EXPECT(assert_equal(V_l3_move, twists.at("link_3")));
}

// test fk for a four bar linkage (loopy)
TEST(forwardKinematics, four_bar) {
  Robot four_bar = CreateRobotFromFile(std::string(SDF_PATH) +
                                       "/test/four_bar_linkage_pure.sdf");
  four_bar.getLinkByName("l1")->fix();

  JointValues joint_angles, joint_vels;
  for (JointSharedPtr joint : four_bar.joints()) {
    joint_angles[joint->name()] = 0;
    joint_vels[joint->name()] = 0;
  }
  FKResults fk_results = four_bar.forwardKinematics(joint_angles, joint_vels);
  LinkPoses poses = fk_results.first;
  LinkTwists twists = fk_results.second;
  gtsam::Vector6 V_4;
  V_4 << 0, 0, 0, 0, 0, 0;
  gtsam::Pose3 T_4(gtsam::Rot3::Rx(-M_PI_2), gtsam::Point3(0, -1, 0));
  EXPECT(assert_equal(V_4, twists.at("l4"), 1e-6));
  // TODO(yetong): check why this error is large
  EXPECT(assert_equal(T_4, poses.at("l4"), 1e-3));

  // incorrect specficiation of joint angles;
  JointValues wrong_angles = joint_angles;
  JointValues wrong_vels = joint_vels;
  wrong_angles["j1"] = 1;
  wrong_vels["j1"] = 1;
  THROWS_EXCEPTION(four_bar.forwardKinematics(wrong_angles, joint_vels));
  THROWS_EXCEPTION(four_bar.forwardKinematics(joint_angles, wrong_vels));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
