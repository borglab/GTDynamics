/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testMinTorqueFactor.cpp
 * @brief Test min torque factor.
 * @Author: Alejandro Escontrela
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

#include "gtdynamics/factors/MinTorqueFactor.h"

using gtsam::assert_equal;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
gtsam::Key torque_key = gtsam::Symbol('t', 1);
}  // namespace example

// Test Torque factor for stationary case
TEST(MinTorqueFactor, error) {
  gtdynamics::MinTorqueFactor factor(example::torque_key, example::cost_model);
  double torque = 20;
  gtsam::Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(torque);
  expected_errors = gtsam::Vector1(400);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::torque_key, torque);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
