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
#include <gtdynamics/universal_robot/HelicalJoint.h>
#include <gtdynamics/universal_robot/PrismaticJoint.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/linear/VectorValues.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector6;

TEST(Robot, four_bar_sdf) {
  // Initialize Robot instance from a file.
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("test/four_bar_linkage.sdf"));

  // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(5, robot.links().size()));
  EXPECT(assert_equal(5, robot.joints().size()));

  // Test link(...) and joint(...)
  EXPECT(assert_equal("l0", robot.link("l0")->name()));
  EXPECT(assert_equal("l1", robot.link("l1")->name()));
  EXPECT(assert_equal("l2", robot.link("l2")->name()));
  EXPECT(assert_equal("l3", robot.link("l3")->name()));
  EXPECT(assert_equal("l4", robot.link("l4")->name()));
  EXPECT(assert_equal("j0", robot.joint("j0")->name()));
  EXPECT(assert_equal("j1", robot.joint("j1")->name()));
  EXPECT(assert_equal("j2", robot.joint("j2")->name()));
  EXPECT(assert_equal("j3", robot.joint("j3")->name()));
  EXPECT(assert_equal("j4", robot.joint("j4")->name()));
}

TEST(Robot, simple_rr_sdf) {
  // Initialize Robot instance from a file.
  auto robot = simple_rr::getRobot();

  // // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(3, robot.links().size()));
  EXPECT(assert_equal(2, robot.joints().size()));

  // Test link(...) and joint(...)
  EXPECT(assert_equal("link_0", robot.link("link_0")->name()));
  EXPECT(assert_equal("link_1", robot.link("link_1")->name()));
  EXPECT(assert_equal("link_2", robot.link("link_2")->name()));

  EXPECT(assert_equal("joint_1", robot.joint("joint_1")->name()));
  EXPECT(assert_equal("joint_2", robot.joint("joint_2")->name()));
}

TEST(Robot, removeLink) {
  // Initialize Robot instance from a file.
  auto robot = four_bar_linkage_pure::getRobot();

  robot.removeLink(robot.link("l2"));
  EXPECT(robot.numLinks() == 3);
  EXPECT(robot.numJoints() == 2);
  EXPECT(robot.link("l1")->joints().size() == 1);
  EXPECT(robot.link("l3")->joints().size() == 1);
}

TEST(Robot, ForwardKinematics) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  Values values;

  // not fixing a link would cause an exception
  THROWS_EXCEPTION(robot.forwardKinematics(values));

  // test fk at rest
  // Values are empty but will default to 0 joint angles.
  robot = robot.fixLink("l1");
  Values results = robot.forwardKinematics(values);

  // The CoM frames at rest are:
  Pose3 T_wl1_rest(Rot3::Identity(), Point3(0, 0, 1));
  Pose3 T_wl2_rest(Rot3::Identity(), Point3(0, 0, 3));
  // At rest, all the twists are 0:
  Vector6 V_l1_rest, V_l2_rest;
  V_l1_rest << 0, 0, 0, 0, 0, 0;
  V_l2_rest << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(T_wl1_rest, Pose(results, 0)));
  EXPECT(assert_equal(T_wl2_rest, Pose(results, 1)));
  EXPECT(assert_equal(V_l1_rest, Twist(results, 0)));
  EXPECT(assert_equal(V_l2_rest, Twist(results, 1)));

  // test fk with moving joint and fixed base
  Values values2;
  InsertJointAngle(&values2, 0, M_PI_2);  // rotate joint by 90 degrees
  InsertJointVel(&values2, 0, 1.0);       // joint velocity is 1 rad/s.

  Values results2 = robot.forwardKinematics(values2);

  Pose3 T_wl1_move(Rot3::Identity(), Point3(0, 0, 1));  // link1 stays put
  // link2 is rotated by 90 degrees and now points along -Y axis.
  Pose3 T_wl2_move(Rot3::Rx(M_PI_2), Point3(0, -1, 2));
  Vector6 V_l1_move, V_l2_move;
  V_l1_move << 0, 0, 0, 0, 0, 0;
  V_l2_move << 1, 0, 0, 0, -1, 0;
  EXPECT(assert_equal(T_wl1_move, Pose(results2, 0)));
  EXPECT(assert_equal(T_wl2_move, Pose(results2, 1)));
  EXPECT(assert_equal(V_l1_move, Twist(results2, 0)));
  EXPECT(assert_equal(V_l2_move, Twist(results2, 1)));

  // test fk with moving joint and moving base
  robot = robot.unfixLink("l1");
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

TEST(Robot, ForwardKinematicsRPR) {
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rpr.sdf"), "simple_rpr_sdf");

  Values values;

  // test fk at rest
  robot = robot.fixLink("link_0");
  Values fk_results = robot.forwardKinematics(values);

  Pose3 T_wl0_rest(Rot3::Identity(), Point3(0, 0, 0.1));
  Pose3 T_wl1_rest(Rot3::Identity(), Point3(0, 0, 0.5));
  Pose3 T_wl2_rest(Rot3::Identity(), Point3(0, 0, 1.1));
  Pose3 T_wl3_rest(Rot3::Identity(), Point3(0, 0, 1.7));
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

  Pose3 T_wl0_move(Rot3::Identity(), Point3(0, 0, 0.1));
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

// Replacing a joint keeps every link's joint count the same, but the cached
// traversal must still be refreshed so FK uses the new joint.
TEST(ForwardKinematics, ReplaceJoint) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  robot = robot.fixLink("l1");
  auto l1 = robot.link("l1"), l2 = robot.link("l2");
  auto old_joint = robot.joint("j1");

  // Populate the cache using the original joint (id 0, about the x axis).
  Values values;
  InsertJointAngle(&values, 0, M_PI_2);
  InsertJointAngle(&values, 1, M_PI_2);
  Values results = robot.forwardKinematics(values);
  EXPECT(assert_equal(Pose3(Rot3::Rx(M_PI_2), Point3(0, -1, 2)),
                      Pose(results, 1)));

  // Swap in a joint with a different id, rotating about the y axis.
  auto new_joint = std::make_shared<RevoluteJoint>(
      1, "j2", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, gtsam::Vector3(0, 1, 0));
  for (auto &&link : {l1, l2}) {
    link->removeJoint(old_joint);
    link->addJoint(new_joint);
  }

  results = robot.forwardKinematics(values);
  EXPECT(assert_equal(Pose3(Rot3::Ry(M_PI_2), Point3(1, 0, 2)),
                      Pose(results, 1)));
}

// test fk for a four bar linkage (loopy)
TEST(ForwardKinematics, FourBar) {
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));
  robot = robot.fixLink("l1");

  Values values;
  Values fk_results = robot.forwardKinematics(values);

  Vector6 V_4;
  V_4 << 0, 0, 0, 0, 0, 0;
  Pose3 T_4(Rot3::Rx(-M_PI_2), Point3(0, -1, 0));
  EXPECT(assert_equal(V_4, Twist(fk_results, 3), 1e-6));
  // TODO(yetong): check why this error is large
  EXPECT(assert_equal(T_4, Pose(fk_results, 3), 1e-3));

  // incorrect specficiation of joint angles.
  Values wrong_angles = values;
  InsertJointAngle(&wrong_angles, 0, 1.0);
  THROWS_EXCEPTION(robot.forwardKinematics(wrong_angles));

  // incorrect specficiation of joint velocites.
  Values wrong_vels = values;
  InsertJointVel(&wrong_vels, 0, 1.0);
  THROWS_EXCEPTION(robot.forwardKinematics(wrong_vels));
}

// FK for a four bar linkage, with an extra link l5 hanging off l3, when the
// pose of an intermediate link is known. Known links are not expanded, so if
// the cached traversal from l1 reaches l3 through the known link, l3 and l5
// must instead be found through the other side of the loop.
TEST(ForwardKinematics, FourBarKnownLink) {
  // Each robot is created fresh, so its traversal cache starts empty.
  auto fourBarWithTail = []() {
    const Robot four_bar = CreateRobotFromFile(
        kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));
    const auto l3 = four_bar.link("l3");
    const auto l5 = std::make_shared<Link>(
        4, "l5", 1.0, gtsam::I_3x3,
        l3->bMcom() * Pose3(Rot3(), Point3(0, 0, 1)), Pose3());
    const auto j5 = std::make_shared<RevoluteJoint>(
        4, "j5", l3->bMcom(), l3, l5, gtsam::Vector3(1, 0, 0));
    l3->addJoint(j5);
    l5->addJoint(j5);

    LinkMap links{{"l5", l5}};
    for (auto &&link : four_bar.links()) links.emplace(link->name(), link);
    JointMap joints{{"j5", j5}};
    for (auto &&joint : four_bar.joints()) joints.emplace(joint->name(), joint);
    return Robot(links, joints).fixLink("l1");
  };

  Values joint_angles;
  InsertJointAngle(&joint_angles, 4, 0.3);
  InsertJointVel(&joint_angles, 4, 0.5);
  const Values expected = fourBarWithTail().forwardKinematics(joint_angles);

  // l2 and l4 are both next to the root, and one of them is l3's parent in
  // the cached traversal.
  for (const std::string name : {"l2", "l4"}) {
    Robot robot = fourBarWithTail();
    const size_t i = robot.link(name)->id();
    Values known = joint_angles;
    InsertPose(&known, i, Pose(expected, i));
    InsertTwist(&known, i, Twist(expected, i));

    // Run on the first (uncached) call and on the cached call.
    for (size_t call = 0; call < 2; ++call) {
      const Values results = robot.forwardKinematics(known);
      for (auto &&link : robot.links()) {
        const size_t j = link->id();
        EXPECT(results.exists(PoseKey(j)));
        if (!results.exists(PoseKey(j))) continue;
        EXPECT(assert_equal(Pose(expected, j), Pose(results, j), 1e-4));
        EXPECT(assert_equal(Twist(expected, j), Twist(results, j), 1e-4));
      }
    }

    // A known pose inconsistent with the joint angles should throw.
    Robot robot2 = fourBarWithTail();
    Values wrong = joint_angles;
    InsertPose(&wrong, i, Pose(expected, i) * Pose3(Rot3(), Point3(0, 0, 0.1)));
    InsertTwist(&wrong, i, Twist(expected, i));
    THROWS_EXCEPTION(robot2.forwardKinematics(wrong));
    THROWS_EXCEPTION(robot2.forwardKinematics(wrong));
  }
}

TEST(ForwardKinematics, A1) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("a1/a1.urdf"), "", true);
  robot = robot.fixLink("trunk");

  Values values;
  Values fk_results = robot.forwardKinematics(values);
  // 21 joint angles, 21, joint velocities, 22 link poses, 22 link twists
  EXPECT_LONGS_EQUAL(86, fk_results.size());

  Values joint_angles;
  // Sanity check that 0 joint angles gives us the same pose as from the URDF
  std::vector<double> angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  size_t jidx = 0;
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&joint_angles, joint->id(), 0, angles[jidx]);
    jidx += 1;
  }

  // This is the toe joint we wish to test.
  size_t joint_id = 20;

  fk_results = robot.forwardKinematics(joint_angles, 0, std::string("trunk"));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(-0.183, -0.13205, -0.4)),
                      Pose(fk_results, joint_id, 0)));

  // Joint angles from A1 simulation.
  angles = {
      0.000174304, 0,        0.923033, -1.83381,    0,           0.000172539,
      0,           0.924125, -1.83302, 0,           0.000137167, 0,
      0.878277,    -1.85284, 0,        0.000140037, 0,           0.877832,
      -1.852,      0,        0};

  jidx = 0;
  joint_angles.clear();
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&joint_angles, joint->id(), 0, angles[jidx]);
    jidx += 1;
  }
  fk_results = robot.forwardKinematics(joint_angles, 0, std::string("trunk"));
  // regression
  EXPECT(assert_equal(
      Pose3(Rot3(0.638821, 0, 0.769356, 0, 1, 0, -0.769356, 0, 0.638821),
            Point3(-0.336871, -0.13205, -0.327764)),
      Pose(fk_results, 20, 0), 1e-6));
}

TEST(Robot, Equality) {
  Robot robot1 = CreateRobotFromFile(
      kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));
  Robot robot2 = CreateRobotFromFile(
      kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));

  EXPECT(robot1 == robot2);
  EXPECT(robot1.equals(robot2));

  // Check if not-equal works as expecred
  JointSharedPtr j = robot1.joints()[0];
  // Set the joint's parent link to default
  *(j->parent()) = Link();

  // robot1 should no longer equal robot2
  EXPECT(!robot1.equals(robot2));
}

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION

// Declaration needed for serialization of derived class.
BOOST_CLASS_EXPORT(gtdynamics::RevoluteJoint)
BOOST_CLASS_EXPORT(gtdynamics::HelicalJoint)
BOOST_CLASS_EXPORT(gtdynamics::PrismaticJoint)

TEST(Robot, Serialization) {
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("test/four_bar_linkage_pure.sdf"));

  using namespace gtsam::serializationTestHelpers;
  EXPECT(equalsObj(robot));
  EXPECT(equalsXML(robot));
  EXPECT(equalsBinary(robot));
}
#endif

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
