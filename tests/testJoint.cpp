/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testJoint.cpp
 * @brief Direct tests for Joint methods and Jacobians.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <vector>

using namespace gtdynamics;

using gtsam::assert_equal;
using gtsam::Matrix61;
using gtsam::Pose3;

TEST(Joint, ParentTchildJacobianMatchesNumerical) {
  auto robot = simple_urdf::getRobot();
  auto joint = robot.joint("j1");

  const double q = -0.73;

  auto parentTchild = [&](double angle) { return joint->parentTchild(angle); };
  const Matrix61 numerical_H_q =
      gtsam::numericalDerivative11<Pose3, double>(parentTchild, q);

  Matrix61 analytic_H_q;
  EXPECT(assert_equal(joint->parentTchild(q), joint->parentTchild(q, analytic_H_q),
                      1e-12));
  EXPECT(assert_equal(numerical_H_q, analytic_H_q, 1e-8));
}

TEST(Joint, ParentTchildJacobianEqualsAxisForOneDoF) {
  auto robot = simple_urdf::getRobot();
  auto joint = robot.joint("j1");

  const std::vector<double> test_angles{-1.1, -0.4, 0.0, 0.7, 1.2};
  for (double q : test_angles) {
    Matrix61 H_q;
    joint->parentTchild(q, H_q);
    EXPECT(assert_equal(joint->cScrewAxis(), H_q, 1e-12));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
