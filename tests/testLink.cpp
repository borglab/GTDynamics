/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testLink.cpp
 * @brief Test Link class.
 * @Author: Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/ScrewJointBase.h"
#include "gtdynamics/utils/utils.h"

using gtdynamics::get_sdf, gtdynamics::Link, gtdynamics::Joint;
using gtsam::assert_equal;

/**
 * Construct a Link via urdf link and ensure all values are as expected.
 */
TEST(Link, urdf_constructor) {
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  // Initialize Robot instance using urdf::ModelInterfacePtr.
  gtdynamics::LinkSharedPtr l1 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l1")));
  gtdynamics::LinkSharedPtr l2 =
      std::make_shared<Link>(Link(*simple_urdf.LinkByName("l2")));
  gtdynamics::JointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = gtdynamics::Joint::JointEffortType::Actuated;

  // Test constructor.
  gtdynamics::RevoluteJointSharedPtr j1 =
      std::make_shared<gtdynamics::RevoluteJoint>(gtdynamics::RevoluteJoint(
          *simple_urdf.JointByName("j1"), j1_params.jointEffortType,
          j1_params.springCoefficient, j1_params.jointLimitThreshold,
          j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
          j1_params.accelerationLimitThreshold, j1_params.torqueLimitThreshold,
          l1, l2));

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
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)),
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
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)),
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
 * Construct the same link via Params and ensure all values are as expected.
 */
TEST(Link, params_constructor) {
  gtdynamics::Link::Params params;
  params.mass = 100;
  params.name = "l1";
  params.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  params.wTl = gtsam::Pose3();
  params.lTcom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1));

  gtdynamics::LinkSharedPtr l1 = std::make_shared<Link>(Link(params));

  // name
  EXPECT(assert_equal("l1", l1->name()));

  // mass
  EXPECT(assert_equal(100, l1->mass()));

  // Check center of mass.
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)),
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
  EXPECT(assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)),
                      l1->lTcom()));

  // Check that no child links/joints have yet been added.
  EXPECT(assert_equal(0, l1->getJoints().size()));
}

TEST(Link, sdf_constructor) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  Link l0 = Link(*model.LinkByName("link_0"));
  Link l1 = Link(*model.LinkByName("link_1"));

  // Both link frames are defined in the world frame.
  EXPECT(assert_equal(gtsam::Pose3::identity(), l0.wTl()));
  EXPECT(assert_equal(gtsam::Pose3::identity(), l1.wTl()));

  // Verify center of mass defined in the link frame is correct.
  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1)),
      l0.lTcom()));
  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.5)),
      l1.lTcom()));

  // Verify center of mass defined in the world frame is correct.
  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1)),
      l0.wTcom()));
  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.5)),
      l1.wTcom()));

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
