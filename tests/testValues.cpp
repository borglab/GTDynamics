/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testRobot.cpp
 * @brief Test Robot instance methods and integration test with various
 * URDF/SDF configurations.
 * @author Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using gtsam::assert_equal;

TEST(Values, at) {
  gtsam::Values values;

  // Check that an exception is thrown if key does not exist
  CHECK_EXCEPTION(JointAngle(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(JointVel(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(JointAccel(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(Torque(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(Pose(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(Twist(values, 7), KeyDoesNotExist);
  CHECK_EXCEPTION(TwistAccel(values, 7), KeyDoesNotExist);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
