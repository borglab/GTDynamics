/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNoiseModelBiasFactor.cpp
 * @brief Test NoiseModel Bias Factor.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "constrainedExample.h"
#include "gtdynamics/optimizer/NoiseModelBiasFactor.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;

/** Create NoiseModelBiasFactor by adding bias to an expression factor, and
 * check derivatives. */
TEST(NoiseModelBiasFactor, derivative) {
  using namespace constrained_example;
  Values values;
  values.insert(x1_key, -0.2);
  values.insert(x2_key, -0.2);

  // create bias factor
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  NoiseModelFactor::shared_ptr original_factor = NoiseModelFactor::shared_ptr(
      new ExpressionFactor<double>(cost_noise, 0., constraint1_expr));
  Vector bias = Vector::Constant(1, 0.2);
  auto bias_factor = NoiseModelBiasFactor(original_factor, bias);

  // check error computation
  double expected_error = 1.4112;  // 0.5*||-0.368 + 0.2||_(0.1^2)^2
  EXPECT(assert_equal(expected_error, bias_factor.error(values)));

  // check jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(bias_factor, values, 1e-7, 1e-5);

  // check cloneWithNewNoiseModel
  auto new_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.5);
  auto new_factor = bias_factor.cloneWithNewNoiseModel(new_noise);
  double expected_new_error = 0.056448;  // 0.5*||-0.368 + 0.2||_(0.5^2)^2
  EXPECT(assert_equal(expected_new_error, new_factor->error(values)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
