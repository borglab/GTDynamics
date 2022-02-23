/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testMotionModelFactor.cpp
 * @brief Test MotionModelFactor.
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

#include "gtdynamics/factors/MotionModelFactor.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

auto kModel = noiseModel::Isotropic::Sigma(6, 0.1);

auto robot = simple_rr::getRobot();
size_t t = 0;

// Test should not throw an exception.
TEST(MotionModelFactor, Constructor) {
  MotionModelFactor(0, 1, 2, 3, kModel, Pose3());
}

TEST(MotionModelFactor, Error) {
  auto link0 = robot.link("link_0");
  auto link1 = robot.link("link_1");
  gtsam::Key l0_0 = PoseKey(link0->id(), 0), l0_1 = PoseKey(link0->id(), 1);
  gtsam::Key l1_0 = PoseKey(link1->id(), 0), l1_1 = PoseKey(link1->id(), 1);
  MotionModelFactor factor(l0_0, l1_0, l0_1, l1_1, kModel, Pose3());

  // Error at rest
  Vector error = factor.evaluateError(link0->bMcom(), link1->bMcom(),
                                      link0->bMcom(), link1->bMcom());
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));

  // Error when the elbow is bent to 90 degrees
  MotionModelFactor factor2(l0_0, l1_0, l0_1, l1_1, kModel,
                            Pose3(Rot3::Rz(M_PI), Point3(0, 0, 0)));
  Pose3 wTl1(Rot3::Rz(M_PI), Point3(0, 0, 0.5));
  Vector error2 = factor2.evaluateError(link0->bMcom(), link1->bMcom(),
                                        link0->bMcom(), wTl1);

  EXPECT(assert_equal(Vector::Zero(6), error2, 1e-9));
}

TEST(MotionModelFactor, Jacobians) {
  auto link0 = robot.link("link_0");
  auto link1 = robot.link("link_1");

  gtsam::Key l0_0 = PoseKey(link0->id(), 0), l0_1 = PoseKey(link0->id(), 1);
  gtsam::Key l1_0 = PoseKey(link1->id(), 0), l1_1 = PoseKey(link1->id(), 1);
  MotionModelFactor factor(l0_0, l1_0, l0_1, l1_1, kModel, Pose3());

  Pose3 wTl0 = link0->bMcom(), wTl1 = link1->bMcom();

  Values values;
  InsertPose(&values, link0->id(), 0, wTl0);
  InsertPose(&values, link0->id(), 1, wTl0);
  InsertPose(&values, link1->id(), 0, wTl1);
  InsertPose(&values, link1->id(), 1, wTl1);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  // Non-trivial pose
  double angle = M_PI;
  MotionModelFactor factor2(l0_0, l1_0, l0_1, l1_1, kModel,
                            Pose3(Rot3::Rz(angle), Point3(0, 0, 0)));
  wTl1 = Pose3(Rot3::Rz(angle), Point3(0, 0, 0.5));
  values.clear();
  InsertPose(&values, link0->id(), 0, wTl0);
  InsertPose(&values, link0->id(), 1, wTl0);
  InsertPose(&values, link1->id(), 0, link1->bMcom());
  InsertPose(&values, link1->id(), 1, wTl1);

  Vector error2 = factor2.evaluateError(wTl0, link1->bMcom(), wTl0, wTl1);
  EXPECT(assert_equal(Vector::Zero(6), error2, 1e-9));

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor2, values, 1e-7, 1e-3);
}

TEST(MotionModelFactor, ArbitraryTime) {
  size_t t = 81;
  auto link0 = robot.links()[0];
  auto link1 = robot.links()[1];

  // Non-trivial joint angle
  double angle = M_PI;
  MotionModelFactor factor(kModel, link0, link1,
                           Pose3(Rot3::Rz(angle), Point3(0, 0, 0)), t, t + 1);

  Pose3 wTl0 = link0->bMcom();
  Pose3 wTl1_t0 = link1->bMcom(),
        wTl1_t1 = Pose3(Rot3::Rz(angle), gtsam::Point3(0, 0, 0.5));

  Values values;
  InsertPose(&values, link0->id(), t, wTl0);
  InsertPose(&values, link1->id(), t, wTl1_t0);
  InsertPose(&values, link0->id(), t+1, wTl0);
  InsertPose(&values, link1->id(), t+1, wTl1_t1);

  Vector error = factor.evaluateError(wTl0, wTl1_t0, wTl0, wTl1_t1);
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-9));

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
