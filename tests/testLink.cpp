/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testLink.cpp
 * @brief Test Link class.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

// Construct the same link via Params and ensure all values are as expected.
TEST(Link, params_constructor) {
  Link::Params parameters;
  parameters.mass = 100;
  parameters.name = "l1";
  parameters.inertia = gtsam::Vector3(3, 2, 1).asDiagonal();
  parameters.wTl = Pose3();
  parameters.lTcom = Pose3(Rot3(), Point3(0, 0, 1));

  LinkSharedPtr l1 = boost::make_shared<Link>(parameters);

  // name
  EXPECT(assert_equal("l1", l1->name()));

  // mass
  EXPECT(assert_equal(100, l1->mass()));

  // Check center of mass.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->lTcom()));

  // Check inertia.
  EXPECT(assert_equal(
      (gtsam::Matrix(3, 3) << 3, 0, 0, 0, 2, 0, 0, 0, 1).finished(),
      l1->inertia()));

  // Check general mass matrix.
  EXPECT(assert_equal((gtsam::Matrix(6, 6) << 3, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
                       0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100,
                       0, 0, 0, 0, 0, 0, 100)
                          .finished(),
                      l1->inertiaMatrix()));

  // Assert correct center of mass in link frame.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1->lTcom()));

  // Check that no child links/joints have yet been added.
  EXPECT(assert_equal(0, l1->getJoints().size()));
}

// Check the links in the simple RR robot.
TEST(Link, sdf_constructor) {
  auto model =
      get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  Link l0 = Link(*model.LinkByName("link_0"));
  Link l1 = Link(*model.LinkByName("link_1"));

  // Both link frames are defined in the world frame.
  EXPECT(assert_equal(Pose3(), l0.wTl()));
  EXPECT(assert_equal(Pose3(), l1.wTl()));

  // Verify center of mass defined in the link frame is correct.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.1)), l0.lTcom()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.5)), l1.lTcom()));

  // Verify center of mass defined in the world frame is correct.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.1)), l0.wTcom()));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 0.5)), l1.wTcom()));

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
