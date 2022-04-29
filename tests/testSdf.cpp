/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSdf.cpp
 * @brief Test functions in sdf.cpp.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang,
 * Disha Das, Stephanie McCormick
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/universal_robot/HelicalJoint.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/universal_robot/PrismaticJoint.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/universal_robot/sdf_internal.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;

// Load a URDF file and ensure its joints and links were parsed correctly.
TEST(Sdf, load_and_parse_urdf_file) {
  // Load the file and parse URDF structure.
  auto simple_urdf = GetSdf(kUrdfPath + std::string("test/simple_urdf.urdf"));

  // Check that physical and inertial properties were properly parsed..
  EXPECT(assert_equal(2, simple_urdf.LinkCount()));
  EXPECT(assert_equal(1, simple_urdf.JointCount()));

  EXPECT(assert_equal(
      100, simple_urdf.LinkByName("l1")->Inertial().MassMatrix().Mass()));
  EXPECT(assert_equal(
      15, simple_urdf.LinkByName("l2")->Inertial().MassMatrix().Mass()));

  EXPECT(assert_equal(3, simple_urdf.LinkByName("l1")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l1")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(1, simple_urdf.LinkByName("l1")->Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(1, simple_urdf.LinkByName("l2")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l2")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(3, simple_urdf.LinkByName("l2")->Inertial().Moi()(2, 2)));
}

TEST(Sdf, load_and_parse_sdf_file) {
  auto simple_sdf = GetSdf(kSdfPath + std::string("test/simple.sdf"));

  EXPECT(assert_equal(1, simple_sdf.LinkCount()));
  EXPECT(assert_equal(0, simple_sdf.JointCount()));
}

TEST(Sdf, load_and_parse_sdf_world_file) {
  auto simple_sdf =
      GetSdf(kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");

  EXPECT(assert_equal(3, simple_sdf.LinkCount()));
  EXPECT(assert_equal(2, simple_sdf.JointCount()));

  sdf::Link l0 = *simple_sdf.LinkByName("link_0");
  sdf::Link l1 = *simple_sdf.LinkByName("link_1");

  EXPECT(assert_equal(0.05, l0.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l0.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l0.Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(0.05, l1.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l1.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l1.Inertial().Moi()(2, 2)));
}

TEST(Sdf, Pose3FromIgnition) {
  ignition::math::Pose3d pose_to_parse(-1, 1, -1, M_PI / 2, 0, -M_PI);

  gtsam::Pose3 parsed_pose = Pose3FromIgnition(pose_to_parse);

  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 2, 0, -M_PI),
                                   gtsam::Point3(-1, 1, -1)),
                      parsed_pose));
}

/**
 * Test parsing of all joint parameters from URDF or SDF files.
 */
TEST(Sdf, parameters_from_file) {
  // Test for reading parameters from a simple URDF.
  auto simple_urdf = GetSdf(kUrdfPath + std::string("test/simple_urdf.urdf"));
  auto j1_parameters = ParametersFromSdfJoint(*simple_urdf.JointByName("j1"));

  EXPECT(assert_equal(-1.57, j1_parameters.scalar_limits.value_lower_limit));
  EXPECT(assert_equal(1.57, j1_parameters.scalar_limits.value_upper_limit));
  EXPECT(assert_equal(500, j1_parameters.damping_coefficient));
  EXPECT(assert_equal(0.5, j1_parameters.velocity_limit));
  EXPECT(assert_equal(1000.0, j1_parameters.torque_limit));

  // Test for reading parameters (damping coefficient, velocity limit, and
  // torque limit) from a simple SDF.
  auto simple_sdf =
      GetSdf(kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");
  auto joint_1_parameters =
      ParametersFromSdfJoint(*simple_sdf.JointByName("joint_1"));

  EXPECT(assert_equal(0.5, joint_1_parameters.damping_coefficient));
  EXPECT(assert_equal(10, joint_1_parameters.velocity_limit));
  EXPECT(assert_equal(300, joint_1_parameters.torque_limit));

  // Test for reading parameters (joint limits) from spider.sdf.
  auto spider_sdf = GetSdf(kSdfPath + std::string("spider.sdf"), "spider");
  auto knee_1_parameters =
      ParametersFromSdfJoint(*spider_sdf.JointByName("knee_1"));

  EXPECT(assert_equal(-0.349066,
                      knee_1_parameters.scalar_limits.value_lower_limit));
  EXPECT(
      assert_equal(2.44346, knee_1_parameters.scalar_limits.value_upper_limit));
}

/**
 * Test parsing of all joint parameters from a robot created via URDF or SDF
 * files.
 */
TEST(Sdf, create_robot_from_file) {
  // Test for reading parameters from a robot created via simple URDF.
  auto simple_robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  auto simple_j1 = simple_robot.joint("j1");

  EXPECT(assert_equal(-1.57,
                      simple_j1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(1.57,
                      simple_j1->parameters().scalar_limits.value_upper_limit));
  EXPECT(assert_equal(500, simple_j1->parameters().damping_coefficient));
  EXPECT(assert_equal(0.5, simple_j1->parameters().velocity_limit));
  EXPECT(assert_equal(1000.0, simple_j1->parameters().torque_limit));

  // Test for reading parameters (damping coefficient, velocity limit, and
  // torque limit) from a robot created via simple SDF.
  auto simple_rr_robot = CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");
  auto simple_rr_j1 = simple_rr_robot.joint("joint_1");

  EXPECT(assert_equal(0.5, simple_rr_j1->parameters().damping_coefficient));
  EXPECT(assert_equal(10, simple_rr_j1->parameters().velocity_limit));
  EXPECT(assert_equal(300, simple_rr_j1->parameters().torque_limit));

  // Test for reading parameters (joint limits) from a robot created via
  // spider.sdf.
  auto spider_robot =
      CreateRobotFromFile(kSdfPath + std::string("spider.sdf"), "spider");
  auto spider_knee1 = spider_robot.joint("knee_1");

  EXPECT(assert_equal(
      -0.349066, spider_knee1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(
      2.44346, spider_knee1->parameters().scalar_limits.value_upper_limit));
}

/**
 * Construct a Link via URDF and ensure all values are as expected.
 */
TEST(Urdf, ConstructorLink) {
  auto simple_urdf = GetSdf(kUrdfPath + std::string("test/simple_urdf.urdf"));

  // Initialize Robot instance using urdf::ModelInterfacePtr.
  LinkSharedPtr l1 = LinkFromSdf(1, *simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *simple_urdf.LinkByName("l2"));
  JointParams j1_parameters;
  j1_parameters.effort_type = JointEffortType::Actuated;

  auto sdf_link_l1 = simple_urdf.LinkByName("l1");
  auto sdf_link_l2 = simple_urdf.LinkByName("l2");

  Pose3 bTj =
      GetJointFrame(*simple_urdf.JointByName("j1"), sdf_link_l1, sdf_link_l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  auto j1 = boost::make_shared<RevoluteJoint>(1, "j1", bTj, l1, l2, j1_axis,
                                              j1_parameters);

  // get shared ptr
  EXPECT(l1->shared() == l1);

  // get ID
  EXPECT(l1->id() == 1);

  // name
  EXPECT(assert_equal("l1", l1->name()));

  // mass
  EXPECT(assert_equal(100, l1->mass()));

  // Check center of mass.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->bMcom()));

  // Check inertia.
  EXPECT(assert_equal(
      (gtsam::Matrix(3, 3) << 3, 0, 0, 0, 2, 0, 0, 0, 1).finished(),
      l1->inertia()));

  // Check general mass matrix.
  EXPECT(assert_equal(
      (gtsam::Matrix(6, 6) << 3, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100)
          .finished(),
      l1->inertiaMatrix()));

  // Assert correct center of mass in link frame.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->bMcom()));

  // Check that no child links/joints have yet been added.
  EXPECT(assert_equal(0, l1->numJoints()));

  // add joint
  l1->addJoint(j1);
  EXPECT(assert_equal(1, l1->numJoints()));
  EXPECT(l1->joints()[0] == j1);

  // remove joint
  l1->removeJoint(j1);
  EXPECT(assert_equal(0, l1->numJoints()));
}

/**
 * Construct a Revolute Joint from URDF and ensure all values are as
 expected.
 */
TEST(Sdf, urdf_constructor_revolute) {
  auto simple_urdf = GetSdf(kUrdfPath + std::string("test/simple_urdf.urdf"));

  LinkSharedPtr l1 = LinkFromSdf(1, *simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *simple_urdf.LinkByName("l2"));

  auto sdf_link_l1 = simple_urdf.LinkByName("l1");
  auto sdf_link_l2 = simple_urdf.LinkByName("l2");

  auto j1_parameters = ParametersFromSdfJoint(*simple_urdf.JointByName("j1"));
  j1_parameters.effort_type = JointEffortType::Actuated;

  Pose3 bMj1 =
      GetJointFrame(*simple_urdf.JointByName("j1"), sdf_link_l1, sdf_link_l2);

  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  auto j1 = boost::make_shared<RevoluteJoint>(1, "j1", bMj1, l1, l2, j1_axis,
                                              j1_parameters);

  // get shared ptr
  EXPECT(j1->shared() == j1);

  // get ID
  EXPECT(j1->id() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->type() == Joint::Type::Revolute);

  // joint effort type
  EXPECT(j1->parameters().effort_type == JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  Pose3 M_12(Rot3::Rx(0), Point3(0, 0, 2));
  Pose3 M_21(Rot3::Rx(0), Point3(0, 0, -2));

  EXPECT(assert_equal(M_12, j1->relativePoseOf(l2, 0.0)));
  EXPECT(assert_equal(M_21, j1->relativePoseOf(l1, 0.0)));

  // transform to (rotating -pi/2)
  Pose3 T_12(Rot3::Rx(-M_PI / 2), Point3(0, 1, 1));
  Pose3 T_21(Rot3::Rx(M_PI / 2), Point3(0, 1, -1));
  EXPECT(assert_equal(T_12, j1->relativePoseOf(l2, -M_PI / 2)));
  EXPECT(assert_equal(T_21, j1->relativePoseOf(l1, -M_PI / 2)));

  // screw axis
  gtsam::Vector6 screw_axis_l1, screw_axis_l2;
  screw_axis_l1 << -1, 0, 0, 0, -1, 0;
  screw_axis_l2 << 1, 0, 0, 0, -1, 0;
  EXPECT(assert_equal(screw_axis_l1, j1->screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_l2, j1->screwAxis(l2)));

  // links
  auto links = j1->links();
  EXPECT(links[0] == l1);
  EXPECT(links[1] == l2);

  // parent & child link
  EXPECT(j1->parent() == l1);
  EXPECT(j1->child() == l2);

  // joint limit
  EXPECT(assert_equal(-1.57, j1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(1.57, j1->parameters().scalar_limits.value_upper_limit));
  EXPECT(
      assert_equal(1e-9, j1->parameters().scalar_limits.value_limit_threshold));
}

// Construct a Revolute Joint from SDF and ensure all values are as expected.
TEST(Sdf, sdf_constructor_revolute) {
  auto model =
      GetSdf(kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");

  LinkSharedPtr l0 = LinkFromSdf(0, *model.LinkByName("link_0"));
  LinkSharedPtr l1 = LinkFromSdf(1, *model.LinkByName("link_1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *model.LinkByName("link_2"));

  auto sdf_link_l0 = model.LinkByName("link_0");
  auto sdf_link_l1 = model.LinkByName("link_1");
  auto sdf_link_l2 = model.LinkByName("link_2");

  const Pose3 bMj1 =
      GetJointFrame(*model.JointByName("joint_1"), sdf_link_l0, sdf_link_l1);

  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("joint_1"));

  // constructor for j1
  JointParams j1_parameters;
  j1_parameters.effort_type = JointEffortType::Actuated;
  auto j1 = boost::make_shared<RevoluteJoint>(1, "joint_1", bMj1, l0, l1,
                                              j1_axis, j1_parameters);

  // check screw axis
  gtsam::Vector6 screw_axis_j1_l0, screw_axis_j1_l1;
  screw_axis_j1_l0 << 0, 0, -1, 0, 0, 0;
  screw_axis_j1_l1 << 0, 0, 1, 0, 0, 0;
  EXPECT(assert_equal(screw_axis_j1_l0, j1->screwAxis(l0)));
  EXPECT(assert_equal(screw_axis_j1_l1, j1->screwAxis(l1)));

  // Check transform from l0 com to l1 com at rest and at various angles.
  Pose3 M_01(Rot3::identity(), Point3(0, 0, 0.4));
  Pose3 T_01_neg(Rot3::Rz(-M_PI / 2), Point3(0, 0, 0.4));
  Pose3 T_01_pos(Rot3::Rz(M_PI / 2), Point3(0, 0, 0.4));

  EXPECT(assert_equal(M_01, j1->relativePoseOf(l1, 0.0)));
  EXPECT(assert_equal(T_01_neg, j1->relativePoseOf(l1, -M_PI / 2)));
  EXPECT(assert_equal(T_01_pos, j1->relativePoseOf(l1, M_PI / 2)));

  // constructor for j2
  JointParams j2_parameters;
  j2_parameters.effort_type = JointEffortType::Actuated;

  Pose3 bMj2 =
      GetJointFrame(*model.JointByName("joint_2"), sdf_link_l1, sdf_link_l2);

  const gtsam::Vector3 j2_axis = GetSdfAxis(*model.JointByName("joint_2"));

  auto j2 = boost::make_shared<RevoluteJoint>(2, "joint_2", bMj2, l1, l2,
                                              j2_axis, j2_parameters);

  // check screw axis
  gtsam::Vector6 screw_axis_j2_l1, screw_axis_j2_l2;
  screw_axis_j2_l1 << 0, -1, 0, 0.3, 0, 0;
  screw_axis_j2_l2 << 0, 1, 0, 0.3, 0, 0;
  EXPECT(assert_equal(screw_axis_j2_l1, j2->screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_j2_l2, j2->screwAxis(l2)));

  // Check transform from l1 com to l2 com at rest and at various angles.
  Pose3 M_12(Rot3::identity(), Point3(0, 0, 0.6));
  Pose3 T_12_pi_2(Rot3::Ry(M_PI / 2), Point3(0.3, 0.0, 0.3));
  Pose3 T_12_pi_4(Rot3::Ry(M_PI / 4), Point3(0.2121, 0.0, 0.5121));

  EXPECT(assert_equal(M_12, j2->relativePoseOf(l2, 0.0)));
  EXPECT(assert_equal(T_12_pi_2, j2->relativePoseOf(l2, M_PI / 2.0)));
  EXPECT(assert_equal(T_12_pi_4, j2->relativePoseOf(l2, M_PI / 4.0), 1e-3));
}

// Test parsing of Revolute joint limit values from various robots.
TEST(Sdf, limit_params) {
  // Check revolute joint limits parsed correctly for first test robot.
  auto model = GetSdf(kSdfPath + std::string("test/four_bar_linkage.sdf"));
  LinkSharedPtr l1 = LinkFromSdf(1, *model.LinkByName("l1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *model.LinkByName("l2"));

  // check link and com frames
  EXPECT(assert_equal(Pose3(), l1->bMlink()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->bMcom()));
  Rot3 R_check = Rot3::Rx(1.5708);
  EXPECT(assert_equal(Pose3(R_check, Point3(0, 0, 2)), l2->bMlink(), 1e-3));
  EXPECT(assert_equal(Pose3(R_check, Point3(0, -1, 2)), l2->bMcom(), 1e-3));

  auto sdf_link_l1 = model.LinkByName("l1");
  auto sdf_link_l2 = model.LinkByName("l2");

  auto j1_parameters = ParametersFromSdfJoint(*model.JointByName("j1"));
  j1_parameters.effort_type = JointEffortType::Actuated;

  Pose3 j1_bTj =
      GetJointFrame(*model.JointByName("j1"), sdf_link_l1, sdf_link_l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("j1"));

  auto j1 = boost::make_shared<RevoluteJoint>(1, "j1", j1_bTj, l1, l2, j1_axis,
                                              j1_parameters);

  EXPECT(assert_equal(-1.57, j1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(1.57, j1->parameters().scalar_limits.value_upper_limit));
  EXPECT(
      assert_equal(1e-9, j1->parameters().scalar_limits.value_limit_threshold));

  // Check revolute joint limits parsed correctly for a robot with no limits.
  auto model2 =
      GetSdf(kSdfPath + std::string("test/simple_rr.sdf"), "simple_rr_sdf");

  LinkSharedPtr link_0 = LinkFromSdf(0, *model2.LinkByName("link_0"));
  LinkSharedPtr link_1 = LinkFromSdf(1, *model2.LinkByName("link_1"));

  auto sdf_link_0 = model2.LinkByName("link_0");
  auto sdf_link_1 = model2.LinkByName("link_1");

  auto joint_1_parameters =
      ParametersFromSdfJoint(*model2.JointByName("joint_1"));
  joint_1_parameters.effort_type = JointEffortType::Actuated;

  Pose3 joint_1_bTj =
      GetJointFrame(*model2.JointByName("joint_1"), sdf_link_0, sdf_link_1);
  const gtsam::Vector3 joint_1_axis =
      GetSdfAxis(*model2.JointByName("joint_1"));

  auto joint_1 = boost::make_shared<RevoluteJoint>(1, "joint_1", joint_1_bTj,
                                                   link_0, link_1, joint_1_axis,
                                                   joint_1_parameters);

  EXPECT(assert_equal(-1e16,
                      joint_1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(1e16,
                      joint_1->parameters().scalar_limits.value_upper_limit));
  EXPECT(assert_equal(
      1e-9, joint_1->parameters().scalar_limits.value_limit_threshold));
}

/**
 * Construct a Prismatic joint from URDF and ensure joint type and screw axis
 * are as expected.
 */
TEST(Sdf, urdf_constructor_prismatic) {
  auto simple_urdf =
      GetSdf(kUrdfPath + std::string("test/simple_urdf_prismatic.urdf"));

  LinkSharedPtr l1 = LinkFromSdf(1, *simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *simple_urdf.LinkByName("l2"));

  auto joint1 = *simple_urdf.JointByName("j1");

  auto j1_parameters = ParametersFromSdfJoint(joint1);
  j1_parameters.effort_type = JointEffortType::Actuated;

  Pose3 bTj = GetJointFrame(joint1, simple_urdf.LinkByName("l1"),
                            simple_urdf.LinkByName("l2"));

  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  auto j1 = boost::make_shared<PrismaticJoint>(1, "j1", bTj, l1, l2, j1_axis,
                                               j1_parameters);

  // get shared ptr
  EXPECT(j1->shared() == j1);

  // get ID
  EXPECT(j1->id() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->type() == Joint::Type::Prismatic);

  // joint effort type
  EXPECT(j1->parameters().effort_type == JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  Pose3 M_12(Rot3::Rx(1.5707963268), Point3(0, -1, 1));
  Pose3 M_21(Rot3::Rx(-1.5707963268), Point3(0, -1, -1));
  EXPECT(assert_equal(M_12, j1->relativePoseOf(l2, 0), 1e-5));
  EXPECT(assert_equal(M_21, j1->relativePoseOf(l1, 0), 1e-5));

  // transform to (translating +1)
  Pose3 T_12(Rot3::Rx(1.5707963268), Point3(0, -2, 1));
  Pose3 T_21(Rot3::Rx(-1.5707963268), Point3(0, -1, -2));
  EXPECT(assert_equal(T_12, j1->relativePoseOf(l2, 1), 1e-5));
  EXPECT(assert_equal(T_21, j1->relativePoseOf(l1, 1), 1e-5));

  // screw axis
  gtsam::Vector6 screw_axis_l1, screw_axis_l2;
  screw_axis_l1 << 0, 0, 0, 0, 1, 0;
  screw_axis_l2 << 0, 0, 0, 0, 0, 1;
  EXPECT(assert_equal(screw_axis_l1, j1->screwAxis(l1), 1e-5));
  EXPECT(assert_equal(screw_axis_l2, j1->screwAxis(l2)));

  // links
  auto links = j1->links();
  EXPECT(links[0] == l1);
  EXPECT(links[1] == l2);

  // parent & child link
  EXPECT(j1->parent() == l1);
  EXPECT(j1->child() == l2);

  // joint limit
  EXPECT(assert_equal(0, j1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(2, j1->parameters().scalar_limits.value_upper_limit));
  EXPECT(
      assert_equal(1e-9, j1->parameters().scalar_limits.value_limit_threshold));
}

/**
 * Construct a Screw joint from SDF and ensure joint type and screw axis
 * are as expected.
 */
TEST(Sdf, sdf_constructor_screw) {
  auto model = GetSdf(kSdfPath + std::string("test/simple_screw_joint.sdf"),
                      "simple_screw_joint_sdf");

  LinkSharedPtr l0 = LinkFromSdf(0, *model.LinkByName("link_0"));
  LinkSharedPtr l1 = LinkFromSdf(1, *model.LinkByName("link_1"));

  Pose3 bTj =
      GetJointFrame(*model.JointByName("joint_1"), model.LinkByName("link_0"),
                    model.LinkByName("link_1"));

  // constructor for j1
  JointParams j1_parameters;
  j1_parameters.effort_type = JointEffortType::Actuated;
  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("joint_1"));

  auto j1 = boost::make_shared<HelicalJoint>(
      1, "joint_1", bTj, l0, l1, j1_axis,
      model.JointByName("joint_1")->ThreadPitch(), j1_parameters);

  // expected values for screw about z axis
  // check screw axis
  gtsam::Vector6 screw_axis_j1_l0, screw_axis_j1_l1;
  screw_axis_j1_l0 << 0, 0, -1, 0, 0, -0.5 / 2 / M_PI;  // parent frame
  screw_axis_j1_l1 << 0, 0, 1, 0, 0, 0.5 / 2 / M_PI;    // child frame
  EXPECT(assert_equal(screw_axis_j1_l0, j1->screwAxis(l0)));
  EXPECT(assert_equal(screw_axis_j1_l1, j1->screwAxis(l1)));

  // Check transform from l0 com to l1 com at rest and at various angles.
  Pose3 M_01(Rot3::identity(), Point3(0, 0, 0.4));
  Pose3 T_01_neg(Rot3::Rz(-M_PI / 2), Point3(0, 0, 0.4 - 0.125));
  Pose3 T_01_pos(Rot3::Rz(M_PI / 2), Point3(0, 0, 0.4 + 0.125));

  EXPECT(assert_equal(M_01, j1->relativePoseOf(l1, 0)));
  EXPECT(assert_equal(T_01_neg, j1->relativePoseOf(l1, -M_PI / 2)));
  EXPECT(assert_equal(T_01_pos, j1->relativePoseOf(l1, M_PI / 2)));
}

// Initialize a Robot with "models/urdfs/test/simple_urdf.urdf" and make sure
// that all transforms, link/joint properties, etc. are correct.
TEST(Robot, simple_urdf) {
  // Load urdf file into sdf::Model
  auto simple_urdf = GetSdf(kUrdfPath + std::string("test/simple_urdf.urdf"));

  LinkSharedPtr l1 = LinkFromSdf(1, *simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 = LinkFromSdf(2, *simple_urdf.LinkByName("l2"));

  auto sdf_link_l1 = simple_urdf.LinkByName("l1");
  auto sdf_link_l2 = simple_urdf.LinkByName("l2");

  auto j1_parameters = ParametersFromSdfJoint(*simple_urdf.JointByName("j1"));
  Pose3 bTj =
      GetJointFrame(*simple_urdf.JointByName("j1"), sdf_link_l1, sdf_link_l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  auto j1 = boost::make_shared<RevoluteJoint>(1, "j1", bTj, l1, l2, j1_axis,
                                              j1_parameters);

  // Initialize Robot instance.
  auto simple_robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  EXPECT(assert_equal(1, simple_robot.link("l1")->numJoints()));
  EXPECT(assert_equal(1, simple_robot.link("l2")->numJoints()));
  EXPECT(simple_robot.link("l1")->id() == 0);
  EXPECT(simple_robot.link("l2")->id() == 1);
  EXPECT(simple_robot.joint("j1")->id() == 0);

  // Check that number of links and joints in the Robot instance is
  // correct.
  EXPECT(assert_equal(2, simple_robot.links().size()));
  EXPECT(assert_equal(1, simple_robot.joints().size()));
  EXPECT(simple_robot.numLinks() == 2);
  EXPECT(simple_robot.numJoints() == 1);

  // Check link and joint names.
  EXPECT(assert_equal("l1", simple_robot.link("l1")->name()));
  EXPECT(assert_equal("l2", simple_robot.link("l2")->name()));
  EXPECT(assert_equal("j1", simple_robot.joint("j1")->name()));

  // Check transforms between link CoM frames.
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -2)),
                      j1->relativePoseOf(j1->parent(), 0)));
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2)),
                      j1->relativePoseOf(j1->child(), 0)));

  // Check link frames and com frames are correct
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->bMcom()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 3)), l2->bMcom()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0)), l1->bMlink()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 2)), l2->bMlink()));
}

// Check the links in the simple RR robot.
TEST(Sdf, sdf_constructor) {
  std::string file_path = kSdfPath + std::string("test/simple_rr.sdf");
  std::string model_name = "simple_rr_sdf";
  Link l0 = Link(*LinkFromSdf(0, "link_0", file_path, model_name));
  Link l1 = Link(*LinkFromSdf(1, "link_1", file_path, model_name));

  // Verify center of mass defined in the world frame is correct.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.1)), l0.bMcom()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.5)), l1.bMcom()));

  // Verify link frame of both links is identity.
  EXPECT(assert_equal(Pose3(), l0.bMlink()));
  EXPECT(assert_equal(Pose3(), l1.bMlink()));

  // Verify that mass is correct.
  EXPECT(assert_equal(0.01, l0.mass()));
  EXPECT(assert_equal(0.01, l1.mass()));

  // Verify that inertia elements are correct.
  EXPECT(assert_equal(
      (gtsam::Matrix(3, 3) << 0.05, 0, 0, 0, 0.06, 0, 0, 0, 0.03).finished(),
      l0.inertia()));
  EXPECT(assert_equal(
      (gtsam::Matrix(3, 3) << 0.05, 0, 0, 0, 0.06, 0, 0, 0, 0.03).finished(),
      l1.inertia()));
}

TEST(Sdf, PreserveFixedJoint) {
  // // Get the A1 robot with fixed joints lumped together.
  Robot a1 = CreateRobotFromFile(kUrdfPath + std::string("a1/a1.urdf"));
  EXPECT_LONGS_EQUAL(12, a1.numJoints());

  Robot a1_fixed_joints =
      CreateRobotFromFile(kUrdfPath + std::string("a1/a1.urdf"), "", true);
  EXPECT_LONGS_EQUAL(21, a1_fixed_joints.numJoints());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
