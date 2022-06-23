/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testJointMeasurementFactor.cpp
 * @brief Test JointMeasurementFactor.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "gtdynamics/factors/JointMeasurementFactor.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

const Key key0 = gtdynamics::internal::PoseKey(0),
          key1 = gtdynamics::internal::PoseKey(1);

auto kModel = noiseModel::Isotropic::Sigma(6, 0.1);

auto robot = simple_rr::getRobot();
size_t t = 0;

// Test should not throw an exception.
TEST(JointMeasurementFactor, Constructor) {
  JointMeasurementFactor<RevoluteJoint>(key0, key1, kModel, robot.joints()[0],
                                        0.0, t);
}

TEST(JointMeasurementFactor, Error) {
  JointMeasurementFactor<RevoluteJoint> factor(key0, key1, kModel,
                                               robot.joints()[0], 0.0, t);

  auto link0 = robot.links()[0];
  auto link1 = robot.links()[1];

  // Error at rest
  Vector error = factor.evaluateError(link0->bMcom(), link1->bMcom());
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));

  // Error when the elbow is bent to 90 degrees
  JointMeasurementFactor<RevoluteJoint> factor2(key0, key1, kModel,
                                                robot.joints()[0], M_PI, t);

  Pose3 wTl1(Rot3::Rz(M_PI), gtsam::Point3(0, 0, 0.5));
  Vector error2 = factor2.evaluateError(link0->bMcom(), wTl1);

  EXPECT(assert_equal(Vector::Zero(6), error2, 1e-9));
}

TEST(JointMeasurementFactor, Jacobians) {
  JointMeasurementFactor<RevoluteJoint> factor(key0, key1, kModel,
                                               robot.joints()[0], 0.0, t);

  auto link0 = robot.links()[0];
  auto link1 = robot.links()[1];

  Pose3 wTl0 = link0->bMcom(), wTl1 = link1->bMcom();

  Values values;
  InsertPose(&values, link0->id(), wTl0);
  InsertPose(&values, link1->id(), wTl1);

  // Check Jacobians when joint angle is 0.0
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  // Non-trivial joint angle
  double angle = M_PI;
  JointMeasurementFactor<RevoluteJoint> factor2(key0, key1, kModel,
                                                robot.joints()[0], angle, t);
  wTl1 = Pose3(Rot3::Rz(angle), gtsam::Point3(0, 0, 0.5));
  values.clear();
  InsertPose(&values, link0->id(), wTl0);
  InsertPose(&values, link1->id(), wTl1);

  Vector error2 = factor2.evaluateError(wTl0, wTl1);
  EXPECT(assert_equal(Vector::Zero(6), error2, 1e-9));

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor2, values, 1e-7, 1e-3);
}

TEST(JointMeasurementFactor, ArbitraryTime) {
  size_t t = 81;
  auto link0 = robot.links()[0];
  auto link1 = robot.links()[1];

  // Non-trivial joint angle
  double angle = M_PI;
  JointMeasurementFactor<RevoluteJoint> factor(key0, key1, kModel,
                                               robot.joints()[0], angle, t);

  Pose3 wTl0 = link0->bMcom();
  Pose3 wTl1 = Pose3(Rot3::Rz(angle), gtsam::Point3(0, 0, 0.5));

  Values values;
  InsertPose(&values, link0->id(), wTl0);
  InsertPose(&values, link1->id(), wTl1);

  Vector error = factor.evaluateError(wTl0, wTl1);
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
