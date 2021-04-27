/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPoseFactor.cpp
 * @brief Test Projection Factor PPC.
 * @Author: Yetong Zhang
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

#include "gtdynamics/jumpingrobot/factors/ProjectionFactorPPC.h"

using gtdynamics::CustomProjectionFactor;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Rot3, gtsam::Point3,
    gtsam::Point2, gtsam::Symbol, gtsam::Cal3Bundler, gtsam::Values;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_2x2);
gtsam::Key pose_key = Symbol('p', 0), point_key = Symbol('t', 0),
           k_key = Symbol('k', 0);
}  // namespace example

// Test twist factor for stationary case
TEST(CustomProjectionFactor, error) {
  // create functor
  Pose3 pose = Pose3(Rot3(), Point3(0, 0, 0));
  Point3 point(0, 0, 1);
  Cal3Bundler camera_k(1, 0, 0, 0, 0);

  // Create factor
  Point2 measured(0, 0);
  CustomProjectionFactor factor(measured, example::cost_model,
                                example::pose_key, example::point_key,
                                example::k_key);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose, point, camera_k);

  // check value
  auto expected_errors = (gtsam::Vector(2) << 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::pose_key, pose);
  values.insert(example::point_key, point);
  values.insert(example::k_key, camera_k);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}