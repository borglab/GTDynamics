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
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

// Construct the same link via Params and ensure all values are as expected.
TEST(Link, params_constructor) {
  Link l1(1, "l1", 100.0, gtsam::Vector3(3, 2, 1).asDiagonal(),
          Pose3(Rot3(), Point3(0, 0, 1)));

  // name
  EXPECT(assert_equal("l1", l1.name()));

  // mass
  EXPECT(assert_equal(100, l1.mass()));

  // Check center of mass.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1.bTcom()));

  // Check inertia.
  EXPECT(assert_equal(
      (gtsam::Matrix(3, 3) << 3, 0, 0, 0, 2, 0, 0, 0, 1).finished(),
      l1.inertia()));

  // Check general mass matrix.
  EXPECT(assert_equal(
      (gtsam::Matrix(6, 6) << 3, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100)
          .finished(),
      l1.inertiaMatrix()));

  // Assert correct center of mass in link frame.
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), l1.bTcom()));

  // Check that no child links/joints have yet been added.
  EXPECT(assert_equal(0, l1.joints().size()));
}

TEST(Link, NumJoints) {
  using simple_urdf::robot;
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  EXPECT_LONGS_EQUAL(1, l1->numJoints());

  auto j2 = boost::make_shared<RevoluteJoint>(
      123, "j2", Pose3(Rot3(), Point3(0, 0.5, 2)), l1, l2,
      gtsam::Vector3(1, 0, 0), JointParams());

  l1->addJoint(j2);
  EXPECT_LONGS_EQUAL(2, l1->numJoints());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
