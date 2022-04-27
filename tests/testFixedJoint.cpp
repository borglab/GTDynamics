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

#include "gtdynamics/universal_robot/FixedJoint.h"
#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;

using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

/**
 * Construct a Fixed joint.
 */
TEST(FixedJoint, Constructor) {
  auto robot = simple_urdf::getRobot();
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");
  FixedJoint j(0, "j1", gtsam::Pose3(), l1, l2);
}

TEST(FixedJoint, A1) {
  Robot a1 = CreateRobotFromFile(kUrdfPath + std::string("a1/a1.urdf"), "", true);
  std::string joint_name = "FR_toe_fixed";
  EXPECT(a1.joint("FR_toe_fixed")->name() == "FR_toe_fixed");
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
