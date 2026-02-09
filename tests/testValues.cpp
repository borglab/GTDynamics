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
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

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

TEST(Values, DynamicsValuesFromPrev) {
  gtsam::Values values;
  InsertJointAngle(&values, 0, 0, 1.0);
  InsertJointVel(&values, 0, 0, 2.0);

  gtsam::Values next_values = DynamicsValuesFromPrev(values);
  EXPECT(assert_equal(1.0, JointAngle(next_values, 0, 1)));
  EXPECT(assert_equal(2.0, JointVel(next_values, 0, 1)));

  gtsam::Values jump_values = DynamicsValuesFromPrev(values, 5);
  EXPECT(assert_equal(1.0, JointAngle(jump_values, 0, 5)));
  EXPECT(assert_equal(2.0, JointVel(jump_values, 0, 5)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
