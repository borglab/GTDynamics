/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSdf.cpp
 * @brief Test functions in sdf.cpp.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, Yetong Zhang,
 * Disha Das, Stephanie McCormick
 */

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/PrismaticJoint.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/ScrewJoint.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtdynamics; 
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

/**
 * Test parsing of all joint parameters from URDF or SDF files.
 */
TEST(File, parameters_from_file) {
  // Test for reading parameters from a simple URDF.
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  auto j1_parameters = ParametersFromFile(*simple_urdf.JointByName("j1"));

  EXPECT(assert_equal(-1.57, j1_parameters.joint_lower_limit));
  EXPECT(assert_equal(1.57, j1_parameters.joint_upper_limit));
  EXPECT(assert_equal(500, j1_parameters.damping_coefficient));
  EXPECT(assert_equal(0.5, j1_parameters.velocity_limit));
  EXPECT(assert_equal(1000.0, j1_parameters.torque_limit));

  // Test for reading parameters (damping coefficient, velocity limit, and
  // torque limit) from a simple SDF.
  auto simple_sdf = get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");
  auto joint_1_parameters = ParametersFromFile(*simple_sdf.JointByName("joint_1"));

  EXPECT(assert_equal(0.5, joint_1_parameters.damping_coefficient));
  EXPECT(assert_equal(10, joint_1_parameters.velocity_limit));
  EXPECT(assert_equal(300, joint_1_parameters.torque_limit));

  // Test for reading parameters (joint limits) from spider.sdf.
  auto spider_sdf = get_sdf(std::string(SDF_PATH) + "/test/spider.sdf", "spider");
  auto knee_1_parameters = ParametersFromFile(*spider_sdf.JointByName("knee_1"));

  EXPECT(assert_equal(-0.349066, knee_1_parameters.joint_lower_limit));
  EXPECT(assert_equal(2.44346, knee_1_parameters.joint_upper_limit));
}

/**
 * Construct a Link via URDF and ensure all values are as expected.
 */
TEST(Link, urdf_constructor_link) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  // Initialize Robot instance using urdf::ModelInterfacePtr.
  LinkSharedPtr l1 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l2"));
  ScrewJointBase::Parameters j1_parameters;
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 wTj = GetJointFrame(*simple_urdf.JointByName("j1"), l1, l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  RevoluteJointSharedPtr j1 = std::make_shared<RevoluteJoint>(
      "j1", wTj, l1, l2, j1_parameters, j1_axis);

  // get shared ptr
  EXPECT(l1->getSharedPtr() == l1);

  // // get, set ID
  l1->setID(1);
  EXPECT(l1->getID() == 1);

  // name
  EXPECT(assert_equal("l1", l1->name()));

  // mass
  EXPECT(assert_equal(100, l1->mass()));

  // Check center of mass.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)),
                      l1->lTcom()));

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
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)),
                      l1->lTcom()));

  // Check that no child links/joints have yet been added.
  EXPECT(assert_equal(0, l1->getJoints().size()));

  // add joint
  l1->addJoint(j1);
  EXPECT(assert_equal(1, l1->getJoints().size()));
  EXPECT(l1->getJoints()[0] == j1);

  // remove joint
  l1->removeJoint(j1);
  EXPECT(assert_equal(0, l1->getJoints().size()));
}

/**
 * Construct a Revolute Joint from URDF and ensure all values are as expected.
 */
TEST(Joint, urdf_constructor_revolute) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  LinkSharedPtr l1 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l2"));

  auto j1_parameters = ParametersFromFile(*simple_urdf.JointByName("j1"));
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 j1_wTj = GetJointFrame(*simple_urdf.JointByName("j1"), l1, l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  auto j1 = std::make_shared<RevoluteJoint>(
      "j1", j1_wTj, l1, l2, j1_parameters, j1_axis);

  // get shared ptr
  EXPECT(j1->getSharedPtr() == j1);

  // get, set ID
  j1->setID(1);
  EXPECT(j1->getID() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->jointType() == Joint::JointType::Revolute);

  // joint effort type
  EXPECT(j1->jointEffortType() == Joint::JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  Pose3 T_12comRest(Rot3::Rx(0), Point3(0, 0, 2));
  Pose3 T_21comRest(Rot3::Rx(0), Point3(0, 0, -2));
  EXPECT(assert_equal(T_12comRest, j1->transformFrom(l2)));
  EXPECT(assert_equal(T_21comRest, j1->transformTo(l2)));

  // transform from (rotating -pi/2)
  Pose3 T_12com(Rot3::Rx(-M_PI / 2), Point3(0, 1, 1));
  Pose3 T_21com(Rot3::Rx(M_PI / 2), Point3(0, 1, -1));
  EXPECT(assert_equal(T_12com, j1->transformFrom(l2, -M_PI / 2)));
  EXPECT(assert_equal(T_21com, j1->transformFrom(l1, -M_PI / 2)));

  // transfrom to (rotating -pi/2)
  EXPECT(assert_equal(T_12com, j1->transformTo(l1, -M_PI / 2)));
  EXPECT(assert_equal(T_21com, j1->transformTo(l2, -M_PI / 2)));

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
  EXPECT(j1->parentLink() == l1);
  EXPECT(j1->childLink() == l2);

  // joint limit
  EXPECT(assert_equal(-1.57, j1->jointLowerLimit()));
  EXPECT(assert_equal(1.57, j1->jointUpperLimit()));
  EXPECT(assert_equal(0.0, j1->jointLimitThreshold()));
}


/**
 * Construct a Revolute Joint from SDF and ensure all values are as expected.
 */
TEST(Joint, sdf_constructor_revolute) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  LinkSharedPtr l0 = std::make_shared<Link>(*model.LinkByName("link_0"));
  LinkSharedPtr l1 = std::make_shared<Link>(*model.LinkByName("link_1"));
  LinkSharedPtr l2 = std::make_shared<Link>(*model.LinkByName("link_2"));

  Pose3 j1_wTj = GetJointFrame(*model.JointByName("joint_1"), l0, l1);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("joint_1"));
  
  // constructor for j1
  ScrewJointBase::Parameters j1_parameters;
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;
  auto j1 =
      std::make_shared<RevoluteJoint>(
          "joint_1", j1_wTj, l0, l1, j1_parameters, j1_axis);

  // check screw axis
  gtsam::Vector6 screw_axis_j1_l0, screw_axis_j1_l1;
  screw_axis_j1_l0 << 0, 0, -1, 0, 0, 0;
  screw_axis_j1_l1 << 0, 0, 1, 0, 0, 0;
  EXPECT(assert_equal(screw_axis_j1_l0, j1->screwAxis(l0)));
  EXPECT(assert_equal(screw_axis_j1_l1, j1->screwAxis(l1)));

  // Check transform from l0 com to l1 com at rest and at various angles.
  Pose3 T_01comRest(Rot3::identity(), Point3(0, 0, 0.4));
  Pose3 T_01com_neg(Rot3::Rz(-M_PI / 2),
                           Point3(0, 0, 0.4));
  Pose3 T_01com_pos(Rot3::Rz(M_PI / 2), Point3(0, 0, 0.4));

  EXPECT(assert_equal(T_01comRest, j1->transformTo(l0)));
  EXPECT(assert_equal(T_01com_neg, j1->transformTo(l0, -M_PI / 2)));
  EXPECT(assert_equal(T_01com_pos, j1->transformFrom(l1, M_PI / 2)));

  // constructor for j2
  ScrewJointBase::Parameters j2_parameters;
  j2_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 j2_wTj = GetJointFrame(*model.JointByName("joint_2"), l1, l2);
  const gtsam::Vector3 j2_axis = GetSdfAxis(*model.JointByName("joint_2"));

  auto j2 = std::make_shared<RevoluteJoint>(
          "joint_2", j2_wTj, l1, l2, j2_parameters, j2_axis);

  // check screw axis
  gtsam::Vector6 screw_axis_j2_l1, screw_axis_j2_l2;
  screw_axis_j2_l1 << 0, -1, 0, 0.3, 0, 0;
  screw_axis_j2_l2 << 0, 1, 0, 0.3, 0, 0;
  EXPECT(assert_equal(screw_axis_j2_l1, j2->screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_j2_l2, j2->screwAxis(l2)));

  // Check transform from l1 com to l2 com at rest and at various angles.
  Pose3 T_12com_rest(Rot3::identity(), Point3(0, 0, 0.6));
  Pose3 T_12com_pi_2(Rot3::Ry(M_PI / 2),
                            Point3(0.3, 0.0, 0.3));
  Pose3 T_12com_pi_4(Rot3::Ry(M_PI / 4),
                            Point3(0.2121, 0.0, 0.5121));

  EXPECT(assert_equal(T_12com_rest, j2->transformFrom(l2)));
  EXPECT(assert_equal(T_12com_pi_2, j2->transformFrom(l2, M_PI / 2.0)));
  EXPECT(assert_equal(T_12com_pi_4, j2->transformTo(l1, M_PI / 4.0), 1e-3));
}

/**
 * Test parsing of Revolute joint limit values from various robots.
 */
TEST(Joint, limit_params) {
  // Check revolute joint limits parsed correctly for first test robot.
  auto model = get_sdf(std::string(SDF_PATH) + "/test/four_bar_linkage.sdf");
  LinkSharedPtr l1 = std::make_shared<Link>(*model.LinkByName("l1"));
  LinkSharedPtr l2 = std::make_shared<Link>(*model.LinkByName("l2"));
  auto j1_parameters = ParametersFromFile(*model.JointByName("j1"));
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 j1_wTj = GetJointFrame(*model.JointByName("j1"), l1, l2);
  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("j1"));

  auto j1 = std::make_shared<RevoluteJoint>(
          "j1", j1_wTj, l1, l2, j1_parameters, j1_axis);

  EXPECT(assert_equal(-1.57, j1->jointLowerLimit()));
  EXPECT(assert_equal(1.57, j1->jointUpperLimit()));
  EXPECT(assert_equal(0.0, j1->jointLimitThreshold()));

  // Check revolute joint limits parsed correctly for a robot with no limits.
  auto model2 =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");
  LinkSharedPtr link_0 =
      std::make_shared<Link>(*model2.LinkByName("link_0"));
  LinkSharedPtr link_1 =
      std::make_shared<Link>(*model2.LinkByName("link_1"));
  auto joint_1_parameters = ParametersFromFile(*model2.JointByName("joint_1"));
  joint_1_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 joint_1_wTj = GetJointFrame(*model2.JointByName("joint_1"), link_0, link_1);
  const gtsam::Vector3 joint_1_axis = GetSdfAxis(*model2.JointByName("joint_1"));

  auto joint_1 = std::make_shared<RevoluteJoint>(
          "joint_1", joint_1_wTj, link_0, link_1, joint_1_parameters, joint_1_axis);

  EXPECT(assert_equal(-1e16, joint_1->jointLowerLimit()));
  EXPECT(assert_equal(1e16, joint_1->jointUpperLimit()));
  EXPECT(assert_equal(0.0, joint_1->jointLimitThreshold()));
}

/**
 * Construct a Prismatic joint from URDF and ensure joint type and screw axis
 * are as expected.
 */
TEST(Joint, urdf_constructor_prismatic) {
  auto simple_urdf =
      get_sdf(std::string(URDF_PATH) + "/test/simple_urdf_prismatic.urdf");

  auto l1 = std::make_shared<Link>(*simple_urdf.LinkByName("l1"));
  auto l2 = std::make_shared<Link>(*simple_urdf.LinkByName("l2"));

  auto joint1 = *simple_urdf.JointByName("j1");

  auto j1_parameters = ParametersFromFile(joint1);
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;

  Pose3 wTj = GetJointFrame(joint1, l1, l2);

  const gtsam::Vector3 j1_axis = GetSdfAxis(*simple_urdf.JointByName("j1"));

  // Test constructor.
  auto j1 = std::make_shared<PrismaticJoint>("j1", wTj, l1, l2, j1_parameters, j1_axis);

  // get shared ptr
  EXPECT(j1->getSharedPtr() == j1);

  // get, set ID
  j1->setID(1);
  EXPECT(j1->getID() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->jointType() == Joint::JointType::Prismatic);

  // joint effort type
  EXPECT(j1->jointEffortType() == Joint::JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  Pose3 T_12comRest(Rot3::Rx(1.5707963268),
                           Point3(0, -1, 1));
  Pose3 T_21comRest(Rot3::Rx(-1.5707963268),
                           Point3(0, -1, -1));
  EXPECT(assert_equal(T_12comRest, j1->transformFrom(l2), 1e-5));
  EXPECT(assert_equal(T_21comRest, j1->transformTo(l2), 1e-5));

  // transform from (translating +1)
  Pose3 T_12com(Rot3::Rx(1.5707963268), Point3(0, -2, 1));
  Pose3 T_21com(Rot3::Rx(-1.5707963268),
                       Point3(0, -1, -2));
  EXPECT(assert_equal(T_12com, j1->transformFrom(l2, 1), 1e-5));
  EXPECT(assert_equal(T_21com, j1->transformFrom(l1, 1), 1e-5));

  // transfrom to (translating +1)
  EXPECT(assert_equal(T_12com, j1->transformTo(l1, 1), 1e-5));
  EXPECT(assert_equal(T_21com, j1->transformTo(l2, 1), 1e-5));

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
  EXPECT(j1->parentLink() == l1);
  EXPECT(j1->childLink() == l2);

  // joint limit
  EXPECT(assert_equal(0, j1->jointLowerLimit()));
  EXPECT(assert_equal(2, j1->jointUpperLimit()));
  EXPECT(assert_equal(0.0, j1->jointLimitThreshold()));
}

/**
 * Construct a Screw joint from SDF and ensure joint type and screw axis
 * are as expected.
 */
TEST(Joint, sdf_constructor_screw) {
  auto model = get_sdf(std::string(SDF_PATH) + "/test/simple_screw_joint.sdf",
                       "simple_screw_joint_sdf");

  LinkSharedPtr l0 = std::make_shared<Link>(*model.LinkByName("link_0"));
  LinkSharedPtr l1 = std::make_shared<Link>(*model.LinkByName("link_1"));

  Pose3 wTj = GetJointFrame(*model.JointByName("joint_1"), l0, l1);

  // constructor for j1
  ScrewJointBase::Parameters j1_parameters;
  j1_parameters.effort_type = Joint::JointEffortType::Actuated;
  const gtsam::Vector3 j1_axis = GetSdfAxis(*model.JointByName("joint_1"));

  ScrewJointSharedPtr j1 =
      std::make_shared<ScrewJoint>("joint_1", wTj, l0, l1, j1_parameters,
                                   j1_axis, model.JointByName("joint_1")->ThreadPitch());

  // expected values for screw about z axis
  // check screw axis
  gtsam::Vector6 screw_axis_j1_l0, screw_axis_j1_l1;
  screw_axis_j1_l0 << 0, 0, -1, 0, 0, -0.5 / 2 / M_PI; // parent frame
  screw_axis_j1_l1 << 0, 0, 1, 0, 0, 0.5 / 2 / M_PI;   // child frame
  EXPECT(assert_equal(screw_axis_j1_l0, j1->screwAxis(l0)));
  EXPECT(assert_equal(screw_axis_j1_l1, j1->screwAxis(l1)));

  // Check transform from l0 com to l1 com at rest and at various angles.
  Pose3 T_01comRest(Rot3::identity(), Point3(0, 0, 0.4));
  Pose3 T_01com_neg(Rot3::Rz(-M_PI / 2),
                           Point3(0, 0, 0.4-0.125));
  Pose3 T_01com_pos(Rot3::Rz(M_PI / 2),
                           Point3(0, 0, 0.4+0.125));

  EXPECT(assert_equal(T_01comRest, j1->transformTo(l0)));
  EXPECT(assert_equal(T_01com_neg, j1->transformTo(l0, -M_PI / 2)));
  EXPECT(assert_equal(T_01com_pos, j1->transformFrom(l1, M_PI / 2)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
