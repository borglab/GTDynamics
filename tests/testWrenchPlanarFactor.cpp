/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWrenchPlanarFactor.cpp
 * @brief Test wrench planar factor.
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

#include "gtdynamics/factors/WrenchPlanarFactor.h"

using gtsam::assert_equal;

namespace example {
// nosie model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);
gtsam::Key wrench_key = gtsam::Symbol('W', 1);
}  // namespace example

// Test wrench planar factor for x-axis
TEST(WrenchPlanarFactor, x_axis) {
  // Create all factors
  gtsam::Vector3 planar_axis;
  planar_axis << 1, 0, 0;
  gtdynamics::WrenchPlanarFactor factor(example::wrench_key,
                                        example::cost_model, planar_axis);
  gtsam::Vector wrench = (gtsam::Vector(6) << 1, 2, 3, 4, 5, 6).finished();

  gtsam::Vector3 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(wrench);
  expected_errors << 2, 3, 4;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test wrench planar factor for y-axis
TEST(WrenchPlanarFactor, y_axis) {
  // Create all factors
  gtsam::Vector3 planar_axis;
  planar_axis << 0, 1, 0;
  gtdynamics::WrenchPlanarFactor factor(example::wrench_key,
                                        example::cost_model, planar_axis);
  gtsam::Vector wrench = (gtsam::Vector(6) << 1, 2, 3, 4, 5, 6).finished();

  gtsam::Vector3 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(wrench);
  expected_errors << 1, 3, 5;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test wrench planar factor for z-axis
TEST(WrenchPlanarFactor, z_axis) {
  // Create all factors
  gtsam::Vector3 planar_axis;
  planar_axis << 0, 0, 1;
  gtdynamics::WrenchPlanarFactor factor(example::wrench_key,
                                        example::cost_model, planar_axis);
  gtsam::Vector wrench = (gtsam::Vector(6) << 1, 2, 3, 4, 5, 6).finished();

  gtsam::Vector3 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(wrench);
  expected_errors << 1, 2, 6;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
