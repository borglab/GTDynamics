/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testRevoluteJoint.cpp
 * @brief Test Joint class.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtdynamics; 
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

/**
 * Construct a Revolute joint via Parameters and ensure all values are as expected.
 */
TEST(Joint, params_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  LinkSharedPtr l1 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l1"));
  LinkSharedPtr l2 =
      std::make_shared<Link>(*simple_urdf.LinkByName("l2"));

  ScrewJointBase::Parameters parameters;
  parameters.effort_type = Joint::JointEffortType::Actuated;
  parameters.joint_lower_limit = -1.57;
  parameters.joint_upper_limit = 1.57;
  parameters.joint_limit_threshold = 0;

  const gtsam::Vector3 axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  auto j1 = std::make_shared<RevoluteJoint>(
      "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2,
      parameters, axis);

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
