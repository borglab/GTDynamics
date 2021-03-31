/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWrenchFactors.cpp
 * @brief Test wrench factors.
 * @author Yetong Zhang
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

#include "gtdynamics/factors/WrenchFactors.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {

// R link example
using simple_urdf_zero_inertia::robot;

auto inertia = robot.links()[0]->inertiaMatrix();

gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
int linkId = 0;
gtsam::Key twist_key = internal::TwistKey(linkId),
           twist_accel_key = internal::TwistAccelKey(linkId),
           wrench_1_key = internal::WrenchKey(linkId, 1),
           wrench_2_key = internal::WrenchKey(linkId, 2),
           wrench_3_key = internal::WrenchKey(linkId, 3),
           wrench_4_key = internal::WrenchKey(linkId, 4),
           pKey = internal::PoseKey(linkId);
}  // namespace example

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor2, error_1) {
  // Create all factors
  gtsam::Vector3 gravity;
  gravity << 0, -9.8, 0;

  WrenchFactor2 factor(example::twist_key, example::twist_accel_key,
                       example::wrench_1_key, example::wrench_2_key,
                       example::pKey, example::cost_model, example::inertia,
                       gravity);
  gtsam::Vector twist, twist_accel, wrench_1, wrench_2;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twist_accel = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_1 = (gtsam::Vector(6) << 0, 0, -1, 0, 4.9, 0).finished();
  wrench_2 = (gtsam::Vector(6) << 0, 0, 1, 0, 4.9, 0).finished();
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_1, wrench_2, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_1_key, wrench_1);
  values.insert(example::wrench_2_key, wrench_2);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor3, error_1) {
  // Create all factors
  gtsam::Vector3 gravity;
  gravity << 0, -9.8, 0;

  WrenchFactor3 factor(example::twist_key, example::twist_accel_key,
                       example::wrench_1_key, example::wrench_2_key,
                       example::wrench_3_key, example::pKey,
                       example::cost_model, example::inertia, gravity);
  gtsam::Vector twist, twist_accel, wrench_1, wrench_2, wrench_3;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twist_accel = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_1 = (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished();
  wrench_2 = (gtsam::Vector(6) << 0, 0, 0, 0, 2, 0).finished();
  wrench_3 = (gtsam::Vector(6) << 0, 0, 0, 0, 6.8, 0).finished();
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(twist, twist_accel, wrench_1, wrench_2,
                                       wrench_3, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_1_key, wrench_1);
  values.insert(example::wrench_2_key, wrench_2);
  values.insert(example::wrench_3_key, wrench_3);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor4, error_1) {
  // Create all factors
  gtsam::Vector3 gravity;
  gravity << 0, -9.8, 0;

  WrenchFactor4 factor(
      example::twist_key, example::twist_accel_key, example::wrench_1_key,
      example::wrench_2_key, example::wrench_3_key, example::wrench_4_key,
      example::pKey, example::cost_model, example::inertia, gravity);
  gtsam::Vector twist, twist_accel, wrench_1, wrench_2, wrench_3, wrench_4;
  twist = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twist_accel = (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_1 = (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished();
  wrench_2 = (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished();
  wrench_3 = (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished();
  wrench_4 = (gtsam::Vector(6) << 0, 0, 0, 0, 6.8, 0).finished();
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(twist, twist_accel, wrench_1, wrench_2,
                                       wrench_3, wrench_4, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_1_key, wrench_1);
  values.insert(example::wrench_2_key, wrench_2);
  values.insert(example::wrench_3_key, wrench_3);
  values.insert(example::wrench_4_key, wrench_4);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor, error) {
  // Create all factors
  gtsam::Vector3 gravity;
  gravity << 0, -9.8, 0;
  int id = example::linkId;

  WrenchFactor factor(example::twist_key, example::twist_accel_key,
                      {example::wrench_1_key, example::wrench_2_key,
                       example::wrench_3_key, example::wrench_4_key},
                      example::pKey, example::cost_model, example::inertia,
                      gravity);
  gtsam::Values x;
  InsertTwist(&x, id, (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id, (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertWrench(&x, id, 1, (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 2, (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 3, (gtsam::Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 4, (gtsam::Vector(6) << 0, 0, 0, 0, 6.8, 0).finished());
  InsertPose(&x, id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));

  gtsam::Vector6 actual_errors = factor.unwhitenedError(x);
  gtsam::Vector6 expected_errors = gtsam::Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, 1e-3);
}

// Test wrench factor for non-zero twist case, zero joint angle
TEST(WrenchFactor2, error_2) {
  // Create all factors

  WrenchFactor2 factor(example::twist_key, example::twist_accel_key,
                       example::wrench_1_key, example::wrench_2_key,
                       example::pKey, example::cost_model, example::inertia);

  gtsam::Vector6 twist, twist_accel, wrench_1, wrench_2;
  twist = (gtsam::Vector(6) << 0, 0, 1, 0, 1, 0).finished();
  twist_accel = (gtsam::Vector(6) << 0, 0, 1, 0, 1, 0).finished();
  wrench_1 = (gtsam::Vector(6) << 0, 0, 4, -1, 2, 0).finished();
  wrench_2 = (gtsam::Vector(6) << 0, 0, -4, 0, -1, 0).finished();
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_1, wrench_2, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_1_key, wrench_1);
  values.insert(example::wrench_2_key, wrench_2);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
