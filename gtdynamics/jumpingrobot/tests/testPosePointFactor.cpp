/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPosePointFactor.cpp
 * @brief Test PosePointFactor.
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

#include "gtdynamics/jumpingrobot/factors/PosePointFactor.h"

using gtdynamics::PosePointFactor;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3,
    gtsam::Values, gtsam::Symbol;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);
gtsam::Key pose_key = Symbol('p', 0), point_key = Symbol('t', 0);
}  // namespace example

// Test pose point factor
TEST(PosePointFactor, error) {
  // create functor
  Pose3 pose = Pose3(Rot3(), Point3(-2, 0, 0));
  Point3 point(3, 0, 0);
  Point3 point_local(5, 0, 0);

  // Create factor
  PosePointFactor factor(example::pose_key, example::point_key,
                         example::cost_model, point_local);

  // call evaluateError
  auto actual_errors = factor.evaluateError(pose, point);

  // check value
  auto expected_errors = (gtsam::Vector(3) << 0, 0, 0).finished();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::pose_key, pose);
  values.insert(example::point_key, point);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}