/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTorqueFactor.cpp
 * @brief Test torque factor.
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <TorqueFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

using gtsam::assert_equal;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
gtsam::Key torque_key = gtsam::Symbol('t', 1),
           wrench_key = gtsam::Symbol('F', 1);
}  // namespace example

// Test Torque factor for stationary case
TEST(TorqueFactor, error) {
  // Create all factors
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  manipulator::TorqueFactor factor(example::wrench_key, example::torque_key,
                                   example::cost_model, screw_axis);
  double torque = 20;
  gtsam::Vector wrench = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  gtsam::Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(wrench, torque);
  expected_errors = gtsam::Vector1(0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::torque_key, torque);
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
