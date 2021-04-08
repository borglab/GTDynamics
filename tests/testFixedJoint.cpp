/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testFixedJoint.cpp
 * @brief Test FixedJoint class.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/FixedJoint.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;

using gtsam::assert_equal;
using gtsam::Pose3;
using gtsam::Point3;
using gtsam::Rot3;

/**
 * Construct a Fixed joint.
 */
TEST(FixedJoint, Constructor) {
  using simple_urdf::robot;
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");
  FixedJoint j(0, "j1", gtsam::Pose3(), l1, l2);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
