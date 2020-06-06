/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testScrewJoint.cpp
 * @brief Test ScrewJoint class.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/ScrewJoint.h"
#include "gtdynamics/utils/utils.h"

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>
using namespace gtdynamics; 

using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

/**
 * Construct the same joint via Params and ensure all values are as expected.
 */
TEST(Joint, params_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  LinkSharedPtr l1 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l1")));
  LinkSharedPtr l2 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l2")));

  Joint::Params parameters;
  parameters.name = "j1";
  parameters.joint_type = Joint::JointType::Screw;
  parameters.effort_type = Joint::JointEffortType::Actuated;
  parameters.parent_link = l1;
  parameters.child_link = l2;
  parameters.wTj = Pose3(Rot3(), Point3(0, 0, 2));
  parameters.joint_lower_limit = -1.57;
  parameters.joint_upper_limit = 1.57;
  parameters.joint_limit_threshold = 0;

  ScrewJointSharedPtr j1 =
      std::make_shared<ScrewJoint>(
          ScrewJoint(parameters, gtsam::Vector3(1, 0, 0), 0.5));

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->jointType() == Joint::JointType::Screw);

  // joint effort type
  EXPECT(j1->jointEffortType() == Joint::JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // screw axis
  gtsam::Vector6 screw_axis_l1, screw_axis_l2;
  screw_axis_l1 << -1, 0, 0, -0.5 / 2 / M_PI, -1, 0; // parent frame
  screw_axis_l2 << 1, 0, 0, 0.5 / 2 / M_PI, -1, 0;   // child frame
  EXPECT(assert_equal(screw_axis_l2, j1->screwAxis(l2)));
  EXPECT(assert_equal(screw_axis_l1, j1->screwAxis(l1)));

  // rest transform
  Pose3 T_12comRest(Rot3::Rx(0), Point3(0, 0, 2));
  Pose3 T_21comRest(Rot3::Rx(0), Point3(0, 0, -2));
  EXPECT(assert_equal(T_12comRest, j1->transformFrom(l2)));
  EXPECT(assert_equal(T_21comRest, j1->transformTo(l2)));

  // transform from (rotating -pi/2)
  Pose3 T_12com(Rot3::Rx(-M_PI / 2), Point3(-0.125, 1, 1));
  Pose3 T_21com(Rot3::Rx(M_PI / 2), Point3(0.125, 1, -1));
  EXPECT(assert_equal(T_12com, j1->transformFrom(l2, -M_PI / 2)));
  EXPECT(assert_equal(T_21com, j1->transformFrom(l1, -M_PI / 2)));

  // transfrom to (rotating -pi/2)
  EXPECT(assert_equal(T_12com, j1->transformTo(l1, -M_PI / 2)));
  EXPECT(assert_equal(T_21com, j1->transformTo(l2, -M_PI / 2)));

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

TEST(Joint, sdf_constructor) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_screw_joint.sdf",
              "simple_screw_joint_sdf");

  LinkSharedPtr l0 = std::make_shared<Link>(Link(*model.LinkByName("link_0")));
  LinkSharedPtr l1 = std::make_shared<Link>(Link(*model.LinkByName("link_1")));

  // constructor for j1
  JointParams j1_parameters;
  j1_parameters.name = "j1";
  j1_parameters.jointEffortType = Joint::JointEffortType::Actuated;
  ScrewJointSharedPtr j1 =
      std::make_shared<ScrewJoint>(ScrewJoint(
          *model.JointByName("joint_1"), j1_parameters.jointEffortType,
          j1_parameters.springCoefficient, j1_parameters.jointLimitThreshold,
          j1_parameters.velocityLimitThreshold, j1_parameters.accelerationLimit,
          j1_parameters.accelerationLimitThreshold,
          j1_parameters.torqueLimitThreshold, l0, l1));

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
