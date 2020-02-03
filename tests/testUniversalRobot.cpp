/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testUniversalRobot.cpp
 * @brief Test UniversalRobot instance methods and integration test with various
 * URDF/SDF configurations.
 * @Author Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <UniversalRobot.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>
#include <utils.h>

using gtsam::assert_equal;
using robot::get_sdf, robot::UniversalRobot, robot::LinkJointPair,
    robot::extract_structure_from_sdf;

// Initialize a UniversalRobot with "urdfs/test/simple_urdf.urdf" and make sure
// that all transforms, link/joint properties, etc. are correct.
TEST(UniversalRobot, simple_urdf) {
  // Load urdf file into sdf::Model
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  LinkJointPair links_and_joints = extract_structure_from_sdf(simple_urdf);
  robot::LinkMap name_to_link = links_and_joints.first;
  robot::JointMap name_to_joint = links_and_joints.second;
  EXPECT(assert_equal(2, name_to_link.size()));
  EXPECT(assert_equal(1, name_to_joint.size()));

  robot::RobotLinkConstSharedPtr l1 = name_to_link.at("l1");
  robot::RobotLinkConstSharedPtr l2 = name_to_link.at("l2");
  robot::RobotJointSharedPtr j1 = name_to_joint.at("j1");
  EXPECT(assert_equal(1, l1->getJoints().size()));
  EXPECT(assert_equal(1, l2->getJoints().size()));
  EXPECT(l1->getID() == 0);
  EXPECT(l2->getID() == 1);
  EXPECT(j1->getID() == 0);

  // Initialize UniversalRobot instance using RobotLink and RobotJoint
  // instances.
  UniversalRobot simple_robot = UniversalRobot(links_and_joints);

  // Check that number of links and joints in the UniversalRobot instance is
  // correct.
  EXPECT(assert_equal(2, simple_robot.links().size()));
  EXPECT(assert_equal(1, simple_robot.joints().size()));
  EXPECT(simple_robot.links()[0] == l1);
  EXPECT(simple_robot.links()[1] == l2);
  EXPECT(simple_robot.joints()[0] == j1);

  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -2)),
                      j1->transformTo(j1->childLink())));
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2)),
                      j1->transformFrom(j1->childLink())));
}


TEST(UniversalRobot, four_bar_sdf) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot four_bar =
      UniversalRobot(std::string(SDF_PATH) + "/test/four_bar_linkage.sdf");

  // Check that number of links and joints in the UniversalRobot instance is
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

  EXPECT(assert_equal(-1.57, four_bar.getJointByName("j1")->jointLowerLimit()));
  EXPECT(assert_equal(1.57, four_bar.getJointByName("j1")->jointUpperLimit()));
  EXPECT(
      assert_equal(0.0, four_bar.getJointByName("j1")->jointLimitThreshold()));
}

TEST(UniversalRobot, simple_rr_sdf) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot simple_rr = UniversalRobot(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  // // Check that number of links and joints in the UniversalRobot instance is
  // correct.
  EXPECT(assert_equal(3, simple_rr.links().size()));
  EXPECT(assert_equal(2, simple_rr.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("link_0", simple_rr.getLinkByName("link_0")->name()));
  EXPECT(assert_equal("link_1", simple_rr.getLinkByName("link_1")->name()));
  EXPECT(assert_equal("link_2", simple_rr.getLinkByName("link_2")->name()));

  EXPECT(assert_equal("joint_1", simple_rr.getJointByName("joint_1")->name()));
  EXPECT(assert_equal("joint_2", simple_rr.getJointByName("joint_2")->name()));

  EXPECT(assert_equal(-1e16,
                      simple_rr.getJointByName("joint_1")->jointLowerLimit()));

  EXPECT(assert_equal(1e16,
                      simple_rr.getJointByName("joint_1")->jointUpperLimit()));

  EXPECT(assert_equal(
      0.0, simple_rr.getJointByName("joint_1")->jointLimitThreshold()));
}

TEST(UniversalRobot, removeLink) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot four_bar =
      UniversalRobot(std::string(SDF_PATH) + "/test/four_bar_linkage_pure.sdf");
  four_bar.removeLink(four_bar.getLinkByName("l2"));
  EXPECT(four_bar.numLinks() == 3);
  EXPECT(four_bar.numJoints() == 2);
  EXPECT(four_bar.getLinkByName("l1")->getJoints().size() == 1);
  EXPECT(four_bar.getLinkByName("l3")->getJoints().size() == 1);
}

TEST(UniversalRobot, forwardKinematics) {
  UniversalRobot simple_robot = UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  UniversalRobot::JointValues joint_angles, joint_vels;
  joint_angles["j1"] = 0;
  joint_vels["j1"] = 0;

  // not fixing a link would cause an exception
  THROWS_EXCEPTION(simple_robot.forwardKinematics(joint_angles, joint_vels));

  // test fk at rest
  simple_robot.getLinkByName("l1")->fix();
  UniversalRobot::FKResults fk_results = simple_robot.forwardKinematics(joint_angles, joint_vels);
  UniversalRobot::LinkPoses poses = fk_results.first;
  UniversalRobot::LinkTwists twists = fk_results.second;

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
  fk_results = simple_robot.forwardKinematics(joint_angles, joint_vels, prior_link_name, T_wl1_float, V_l1_float);
  poses = fk_results.first;
  twists = fk_results.second;
  EXPECT(assert_equal(T_wl1_float, poses.at("l1")));
  EXPECT(assert_equal(T_wl2_float, poses.at("l2")));
  EXPECT(assert_equal(V_l1_float, twists.at("l1")));
  EXPECT(assert_equal(V_l2_float, twists.at("l2")));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
