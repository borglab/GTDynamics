/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testRobotJoint.cpp
 * @brief Test RobotJoint class.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <RobotJoint.h>
#include <RobotLink.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>
#include <utils.h>

using robot::get_sdf, robot::RobotJoint, robot::RobotLink,
    robot::RobotJointSharedPtr, robot::RobotLinkSharedPtr;

/**
 * construct a RobotJoint and ensure all values are as expected.
 */
TEST(RobotJoint, urdf_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  RobotLinkSharedPtr l1 =
      std::make_shared<RobotLink>(RobotLink(*simple_urdf.LinkByName("l1")));
  RobotLinkSharedPtr l2 =
      std::make_shared<RobotLink>(RobotLink(*simple_urdf.LinkByName("l2")));

  robot::RobotJointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = robot::RobotJoint::JointEffortType::Actuated;

  // Test constructor.
  RobotJointSharedPtr j1 = std::make_shared<RobotJoint>(
      RobotJoint(*simple_urdf.JointByName("j1"), j1_params.jointEffortType,
                 j1_params.springCoefficient, j1_params.jointLimitThreshold,
                 j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
                 j1_params.accelerationLimitThreshold,
                 j1_params.torqueLimitThreshold, l1, l2->getWeakPtr()));

  // Rest transform is equivalent to transform with q = 0.
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1)), j1->Tjpcom()));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)), j1->Tjccom()));

  // Test joint screw axis
  gtsam::Vector6 screw_axis_j1;
  screw_axis_j1 << 1, 0, 0, 0, -1, 0;
  EXPECT(gtsam::assert_equal(j1->screwAxis(), screw_axis_j1));

  // Test that parent to child link transform is correct for -pi/2 and pi/2.
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::Rx(-M_PI / 2), gtsam::Point3(0, 1, 1)),
      j1->MpcCom(-M_PI / 2)));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::Rx(M_PI / 2), gtsam::Point3(0, -1, 1)),
      j1->MpcCom(M_PI / 2)));

  // Test that ID is set correctly.
  unsigned char id = 'a';
  j1->setID(id);
  EXPECT(gtsam::assert_equal(static_cast<double>(j1->getID()),
                             static_cast<double>(id)));
}

TEST(RobotJoint, sdf_constructor) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  RobotLinkSharedPtr l0 =
      std::make_shared<RobotLink>(RobotLink(*model.LinkByName("link_0")));
  RobotLinkSharedPtr l1 =
      std::make_shared<RobotLink>(RobotLink(*model.LinkByName("link_1")));
  RobotLinkSharedPtr l2 =
      std::make_shared<RobotLink>(RobotLink(*model.LinkByName("link_2")));

  robot::RobotJointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = robot::RobotJoint::JointEffortType::Actuated;
  RobotJointSharedPtr j1 = std::make_shared<RobotJoint>(
      RobotJoint(*model.JointByName("joint_1"), j1_params.jointEffortType,
                 j1_params.springCoefficient, j1_params.jointLimitThreshold,
                 j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
                 j1_params.accelerationLimitThreshold,
                 j1_params.torqueLimitThreshold, l0, l1->getWeakPtr()));

  // Check transform from the joint frame to the child/parent frames.
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.3)),
      j1->Tjccom()));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, -0.1)),
      j1->Tjpcom()));

  // Check that the axis is correctly defined in the joint frame.
  EXPECT(gtsam::assert_equal((gtsam::Vector(3) << 0, 0, 1).finished(),
                             j1->axis()));

  // Check transform from l0 com to l1 com at rest and at various angles.
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.4)),
      j1->MpcCom()));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::Rz(-M_PI / 2), gtsam::Point3(0, 0, 0.4)),
      j1->MpcCom(-M_PI / 2)));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::Rz(M_PI / 2), gtsam::Point3(0, 0, 0.4)),
      j1->MpcCom(M_PI / 2)));

  robot::RobotJointParams j2_params;
  j2_params.name = "j2";
  j2_params.jointEffortType = robot::RobotJoint::JointEffortType::Actuated;
  RobotJointSharedPtr j2 = std::make_shared<RobotJoint>(
      RobotJoint(*model.JointByName("joint_2"), j2_params.jointEffortType,
                 j2_params.springCoefficient, j2_params.jointLimitThreshold,
                 j2_params.velocityLimitThreshold, j2_params.accelerationLimit,
                 j2_params.accelerationLimitThreshold,
                 j2_params.torqueLimitThreshold, l1, l2->getWeakPtr()));

  // Check transform from the joint frame to the child/parent frames.
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.3)),
      j2->Tjccom()));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, -0.3)),
      j2->Tjpcom()));

  // Check that the axis is correctly defined in the joint frame.
  EXPECT(gtsam::assert_equal((gtsam::Vector(3) << 0, 1, 0).finished(),
                             j2->axis()));

  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.6)),
      j2->MpcCom()));

  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3::Ry(M_PI / 2), gtsam::Point3(0.3, 0.0, 0.3)),
      j2->MpcCom(M_PI / 2.0)));

  EXPECT(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3::Ry(M_PI / 4),
                                          gtsam::Point3(0.2121, 0.0, 0.5121)),
                             j2->MpcCom(M_PI / 4.0), 1e-3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
