/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testJoint.cpp
 * @brief Test Joint class.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/utils/utils.h"

using gtdynamics::get_sdf, gtdynamics::Joint, gtdynamics::Link,
    gtdynamics::JointSharedPtr, gtdynamics::LinkSharedPtr;
using gtsam::assert_equal;

/**
 * construct a Joint and ensure all values are as expected.
 */
TEST(Joint, urdf_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  LinkSharedPtr l1 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l1")));
  LinkSharedPtr l2 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l2")));

  gtdynamics::JointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = gtdynamics::Joint::JointEffortType::Actuated;

  // Test constructor.
  JointSharedPtr j1 = std::make_shared<Joint>(
      Joint(*simple_urdf.JointByName("j1"), j1_params.jointEffortType,
            j1_params.springCoefficient, j1_params.jointLimitThreshold,
            j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
            j1_params.accelerationLimitThreshold,
            j1_params.torqueLimitThreshold, l1, l2));

  // get shared ptr
  EXPECT(j1->getSharedPtr() == j1);

  // get, set ID
  j1->setID(1);
  EXPECT(j1->getID() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->jointType() == 'R');

  // joint effort type
  EXPECT(j1->jointEffortType() == gtdynamics::Joint::JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  gtsam::Pose3 T_12comRest(gtsam::Rot3::Rx(0), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 T_21comRest(gtsam::Rot3::Rx(0), gtsam::Point3(0, 0, -2));
  EXPECT(assert_equal(T_12comRest, j1->transformFrom(l2)));
  EXPECT(assert_equal(T_21comRest, j1->transformTo(l2)));

  // transform from (rotating -pi/2)
  gtsam::Pose3 T_12com(gtsam::Rot3::Rx(-M_PI / 2), gtsam::Point3(0, 1, 1));
  gtsam::Pose3 T_21com(gtsam::Rot3::Rx(M_PI / 2), gtsam::Point3(0, 1, -1));
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
 * Construct the same joint via Params and ensure all values are as expected.
 */
TEST(Joint, params_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  LinkSharedPtr l1 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l1")));
  LinkSharedPtr l2 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l2")));

  gtdynamics::Joint::Params params;
  params.name = "j1";
  params.joint_type = 'R';
  params.effor_type = Joint::JointEffortType::Actuated;
  params.parent_link = l1;
  params.child_link = l2;
  params.axis = gtsam::Point3(1, 0, 0);
  params.wTj = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2));
  params.joint_lower_limit = -1.57;
  params.joint_upper_limit = 1.57;
  params.joint_limit_threshold = 0;

  JointSharedPtr j1 = std::make_shared<Joint>(Joint(params));

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint type
  EXPECT(j1->jointType() == 'R');

  // joint effort type
  EXPECT(j1->jointEffortType() == gtdynamics::Joint::JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  gtsam::Pose3 T_12comRest(gtsam::Rot3::Rx(0), gtsam::Point3(0, 0, 2));
  gtsam::Pose3 T_21comRest(gtsam::Rot3::Rx(0), gtsam::Point3(0, 0, -2));
  EXPECT(assert_equal(T_12comRest, j1->transformFrom(l2)));
  EXPECT(assert_equal(T_21comRest, j1->transformTo(l2)));

  // transform from (rotating -pi/2)
  gtsam::Pose3 T_12com(gtsam::Rot3::Rx(-M_PI / 2), gtsam::Point3(0, 1, 1));
  gtsam::Pose3 T_21com(gtsam::Rot3::Rx(M_PI / 2), gtsam::Point3(0, 1, -1));
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

TEST(Joint, sdf_constructor) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  LinkSharedPtr l0 = std::make_shared<Link>(Link(*model.LinkByName("link_0")));
  LinkSharedPtr l1 = std::make_shared<Link>(Link(*model.LinkByName("link_1")));
  LinkSharedPtr l2 = std::make_shared<Link>(Link(*model.LinkByName("link_2")));

  // constructor for j1
  gtdynamics::JointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = gtdynamics::Joint::JointEffortType::Actuated;
  JointSharedPtr j1 = std::make_shared<Joint>(
      Joint(*model.JointByName("joint_1"), j1_params.jointEffortType,
            j1_params.springCoefficient, j1_params.jointLimitThreshold,
            j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
            j1_params.accelerationLimitThreshold,
            j1_params.torqueLimitThreshold, l0, l1));

  // check screw axis
  gtsam::Vector6 screw_axis_j1_l0, screw_axis_j1_l1;
  screw_axis_j1_l0 << 0, 0, -1, 0, 0, 0;
  screw_axis_j1_l1 << 0, 0, 1, 0, 0, 0;
  EXPECT(assert_equal(screw_axis_j1_l0, j1->screwAxis(l0)));
  EXPECT(assert_equal(screw_axis_j1_l1, j1->screwAxis(l1)));

  // Check transform from l0 com to l1 com at rest and at various angles.
  gtsam::Pose3 T_01comRest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.4));
  gtsam::Pose3 T_01com_neg(gtsam::Rot3::Rz(-M_PI / 2),
                           gtsam::Point3(0, 0, 0.4));
  gtsam::Pose3 T_01com_pos(gtsam::Rot3::Rz(M_PI / 2), gtsam::Point3(0, 0, 0.4));

  EXPECT(assert_equal(T_01comRest, j1->transformTo(l0)));
  EXPECT(assert_equal(T_01com_neg, j1->transformTo(l0, -M_PI / 2)));
  EXPECT(assert_equal(T_01com_pos, j1->transformFrom(l1, M_PI / 2)));

  // constructor for j2
  gtdynamics::JointParams j2_params;
  j2_params.name = "j2";
  j2_params.jointEffortType = gtdynamics::Joint::JointEffortType::Actuated;
  JointSharedPtr j2 = std::make_shared<Joint>(
      Joint(*model.JointByName("joint_2"), j2_params.jointEffortType,
            j2_params.springCoefficient, j2_params.jointLimitThreshold,
            j2_params.velocityLimitThreshold, j2_params.accelerationLimit,
            j2_params.accelerationLimitThreshold,
            j2_params.torqueLimitThreshold, l1, l2));

  // check screw axis
  gtsam::Vector6 screw_axis_j2_l1, screw_axis_j2_l2;
  screw_axis_j2_l1 << 0, -1, 0, 0.3, 0, 0;
  screw_axis_j2_l2 << 0, 1, 0, 0.3, 0, 0;
  EXPECT(assert_equal(screw_axis_j2_l1, j2->screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_j2_l2, j2->screwAxis(l2)));

  // Check transform from l1 com to l2 com at rest and at various angles.
  gtsam::Pose3 T_12com_rest(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.6));
  gtsam::Pose3 T_12com_pi_2(gtsam::Rot3::Ry(M_PI / 2),
                            gtsam::Point3(0.3, 0.0, 0.3));
  gtsam::Pose3 T_12com_pi_4(gtsam::Rot3::Ry(M_PI / 4),
                            gtsam::Point3(0.2121, 0.0, 0.5121));

  EXPECT(assert_equal(T_12com_rest, j2->transformFrom(l2)));
  EXPECT(assert_equal(T_12com_pi_2, j2->transformFrom(l2, M_PI / 2.0)));
  EXPECT(assert_equal(T_12com_pi_4, j2->transformTo(l1, M_PI / 4.0), 1e-3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
