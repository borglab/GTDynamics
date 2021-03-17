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
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

// Construct the same link via Params and ensure all values are as expected.
TEST(Link, params_constructor) {
  LinkParams parameters;
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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
