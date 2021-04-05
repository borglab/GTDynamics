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
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector6;

TEST(Robot, four_bar_sdf) {
  // Initialize Robot instance from a file.
  Robot four_bar =
      CreateRobotFromFile(SDF_PATH + "/test/four_bar_linkage.sdf");

  // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(5, four_bar.links().size()));
  EXPECT(assert_equal(5, four_bar.joints().size()));

  // Test link(...) and joint(...)
  EXPECT(assert_equal("l0", four_bar.link("l0")->name()));
  EXPECT(assert_equal("l1", four_bar.link("l1")->name()));
  EXPECT(assert_equal("l2", four_bar.link("l2")->name()));
  EXPECT(assert_equal("l3", four_bar.link("l3")->name()));
  EXPECT(assert_equal("l4", four_bar.link("l4")->name()));
  EXPECT(assert_equal("j0", four_bar.joint("j0")->name()));
  EXPECT(assert_equal("j1", four_bar.joint("j1")->name()));
  EXPECT(assert_equal("j2", four_bar.joint("j2")->name()));
  EXPECT(assert_equal("j3", four_bar.joint("j3")->name()));
  EXPECT(assert_equal("j4", four_bar.joint("j4")->name()));
}

TEST(Robot, simple_rr_sdf) {
  // Initialize Robot instance from a file.
  Robot simple_rr = CreateRobotFromFile(
      SDF_PATH + "/test/simple_rr.sdf", "simple_rr_sdf");

  // // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(3, simple_rr.links().size()));
  EXPECT(assert_equal(2, simple_rr.joints().size()));

  // Test link(...) and joint(...)
  EXPECT(assert_equal("link_0", simple_rr.link("link_0")->name()));
  EXPECT(assert_equal("link_1", simple_rr.link("link_1")->name()));
  EXPECT(assert_equal("link_2", simple_rr.link("link_2")->name()));

  EXPECT(assert_equal("joint_1", simple_rr.joint("joint_1")->name()));
  EXPECT(assert_equal("joint_2", simple_rr.joint("joint_2")->name()));
}

TEST(Robot, removeLink) {
  // Initialize Robot instance from a file.
  Robot four_bar = CreateRobotFromFile(SDF_PATH +
                                       "/test/four_bar_linkage_pure.sdf");
  four_bar.removeLink(four_bar.link("l2"));
  EXPECT(four_bar.numLinks() == 3);
  EXPECT(four_bar.numJoints() == 2);
  EXPECT(four_bar.link("l1")->joints().size() == 1);
  EXPECT(four_bar.link("l3")->joints().size() == 1);
}

TEST(Robot, forwardKinematics) {
  Robot robot =
      CreateRobotFromFile(URDF_PATH + "/test/simple_urdf.urdf");

  Values values;

  // not fixing a link would cause an exception
  THROWS_EXCEPTION(robot.forwardKinematics(values));

  // test fk at rest
  robot.link("l1")->fix();
  Values results = robot.forwardKinematics(values);

  Pose3 T_wl1_rest(Rot3::identity(), Point3(0, 0, 1));
  Pose3 T_wl2_rest(Rot3::identity(), Point3(0, 0, 3));
  Vector6 V_l1_rest, V_l2_rest;
  V_l1_rest << 0, 0, 0, 0, 0, 0;
  V_l2_rest << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(T_wl1_rest, Pose(results, 0)));
  EXPECT(assert_equal(T_wl2_rest, Pose(results, 1)));
  EXPECT(assert_equal(V_l1_rest, Twist(results, 0)));
  EXPECT(assert_equal(V_l2_rest, Twist(results, 1)));

  // test fk with moving joint and fixed base
  Values values2;
  InsertJointAngle(&values2, 0, M_PI_2);
  InsertJointVel(&values2, 0, 1.0);

  Values results2 = robot.forwardKinematics(values2);

  Pose3 T_wl1_move(Rot3::identity(), Point3(0, 0, 1));
  Pose3 T_wl2_move(Rot3::Rx(M_PI_2), Point3(0, -1, 2));
  Vector6 V_l1_move, V_l2_move;
  V_l1_move << 0, 0, 0, 0, 0, 0;
  V_l2_move << 1, 0, 0, 0, -1, 0;
  EXPECT(assert_equal(T_wl1_move, Pose(results2, 0)));
  EXPECT(assert_equal(T_wl2_move, Pose(results2, 1)));
  EXPECT(assert_equal(V_l1_move, Twist(results2, 0)));
  EXPECT(assert_equal(V_l2_move, Twist(results2, 1)));

  // test fk with moving joint and moving base
  robot.link("l1")->unfix();
  Pose3 T_wl1_float(Rot3::Rx(-M_PI_2), Point3(0, 1, 1));
  Pose3 T_wl2_float(Rot3::Rx(0), Point3(0, 2, 2));
  Vector6 V_l1_float, V_l2_float;
  V_l1_float << 1, 0, 0, 0, -1, 0;
  V_l2_float << 2, 0, 0, 0, -2, 2;

  Values values3 = values2;
  InsertPose(&values3, 0, T_wl1_float);
  InsertTwist(&values3, 0, V_l1_float);

  std::string prior_link_name = "l1";
  Values results3 = robot.forwardKinematics(values3, 0, prior_link_name);
  EXPECT(assert_equal(T_wl1_float, Pose(results3, 0)));
  EXPECT(assert_equal(T_wl2_float, Pose(results3, 1)));
  EXPECT(assert_equal(V_l1_float, Twist(results3, 0)));
  EXPECT(assert_equal(V_l2_float, Twist(results3, 1)));
}

TEST(Robot, forwardKinematics_rpr) {
  Robot robot = CreateRobotFromFile(
      SDF_PATH + "/test/simple_rpr.sdf", "simple_rpr_sdf");

  Values values;

  // test fk at rest
  robot.link("link_0")->fix();
  Values fk_results = robot.forwardKinematics(values);

  Pose3 T_wl0_rest(Rot3::identity(), Point3(0, 0, 0.1));
  Pose3 T_wl1_rest(Rot3::identity(), Point3(0, 0, 0.5));
  Pose3 T_wl2_rest(Rot3::identity(), Point3(0, 0, 1.1));
  Pose3 T_wl3_rest(Rot3::identity(), Point3(0, 0, 1.7));
  Vector6 V_l0_rest, V_l1_rest, V_l2_rest, V_l3_rest;
  V_l0_rest << 0, 0, 0, 0, 0, 0;
  V_l1_rest << 0, 0, 0, 0, 0, 0;
  V_l2_rest << 0, 0, 0, 0, 0, 0;
  V_l3_rest << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(T_wl0_rest, Pose(fk_results, 0)));
  EXPECT(assert_equal(T_wl1_rest, Pose(fk_results, 1)));
  EXPECT(assert_equal(T_wl2_rest, Pose(fk_results, 2)));
  EXPECT(assert_equal(T_wl3_rest, Pose(fk_results, 3)));
  EXPECT(assert_equal(V_l0_rest, Twist(fk_results, 0)));
  EXPECT(assert_equal(V_l1_rest, Twist(fk_results, 1)));
  EXPECT(assert_equal(V_l2_rest, Twist(fk_results, 2)));
  EXPECT(assert_equal(V_l3_rest, Twist(fk_results, 3)));

  // test fk with moving prismatic joint and fixed base
  Values values2;
  InsertJointAngle(&values2, 1, M_PI_2);
  InsertJointAngle(&values2, 2, 0.5);
  InsertJointVel(&values2, 2, 1.0);

  fk_results = robot.forwardKinematics(values2);

  Pose3 T_wl0_move(Rot3::identity(), Point3(0, 0, 0.1));
  Pose3 T_wl1_move(Rot3::Ry(M_PI_2), Point3(0.3, 0, 0.2));
  Pose3 T_wl2_move(Rot3::Ry(M_PI_2), Point3(1.4, 0, 0.2));
  Pose3 T_wl3_move(Rot3::Ry(M_PI_2), Point3(2.0, 0, 0.2));
  Vector6 V_l0_move, V_l1_move, V_l2_move, V_l3_move;
  V_l0_move << 0, 0, 0, 0, 0, 0;
  V_l1_move << 0, 0, 0, 0, 0, 0;
  V_l2_move << 0, 0, 0, 0, 0, 1;
  V_l3_move << 0, 0, 0, 0, 0, 1;
  EXPECT(assert_equal(T_wl0_move, Pose(fk_results, 0)));
  EXPECT(assert_equal(T_wl1_move, Pose(fk_results, 1)));
  EXPECT(assert_equal(T_wl2_move, Pose(fk_results, 2)));
  EXPECT(assert_equal(T_wl3_move, Pose(fk_results, 3)));
  EXPECT(assert_equal(V_l0_move, Twist(fk_results, 0)));
  EXPECT(assert_equal(V_l1_move, Twist(fk_results, 1)));
  EXPECT(assert_equal(V_l2_move, Twist(fk_results, 2)));
  EXPECT(assert_equal(V_l3_move, Twist(fk_results, 3)));
}

// test fk for a four bar linkage (loopy)
TEST(forwardKinematics, four_bar) {
  Robot four_bar = CreateRobotFromFile(SDF_PATH +
                                       "/test/four_bar_linkage_pure.sdf");
  four_bar.link("l1")->fix();

  Values values;
  Values fk_results = four_bar.forwardKinematics(values);

  Vector6 V_4;
  V_4 << 0, 0, 0, 0, 0, 0;
  Pose3 T_4(Rot3::Rx(-M_PI_2), Point3(0, -1, 0));
  EXPECT(assert_equal(V_4, Twist(fk_results, 3), 1e-6));
  // TODO(yetong): check why this error is large
  EXPECT(assert_equal(T_4, Pose(fk_results, 3), 1e-3));

  // incorrect specficiation of joint angles.
  Values wrong_angles = values;
  InsertJointAngle(&wrong_angles, 0, 1.0);
  THROWS_EXCEPTION(four_bar.forwardKinematics(wrong_angles));

  // incorrect specficiation of joint velocites.
  Values wrong_vels = values;
  InsertJointVel(&wrong_vels, 0, 1.0);
  THROWS_EXCEPTION(four_bar.forwardKinematics(wrong_vels));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
