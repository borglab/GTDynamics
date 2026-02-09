/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testBiasedFactor.cpp
 * @brief test const bias factor.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/BiasedFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

TEST(BiasedFactor, pose) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Base Factor.
  auto noise = noiseModel::Unit::Create(3);
  auto base_factor = std::make_shared<BetweenFactor<Point3>>(
      x1_key, x2_key, Point3(0, 0, 1), noise);

  // Create manifold values for testing.
  Values values;
  values.insert(x1_key, Point3(0, 0, 0));
  values.insert(x2_key, Point3(1, 0, 1));

  // Bias
  Vector bias = (Vector(3) << 1, 1, 0.1).finished();

  // Test constructor.
  gtdynamics::BiasedFactor const_bias_factor(base_factor, bias);

  // Check error.
  Vector expected_error1 = (Vector(3) << 2, 1, 0.1).finished();
  EXPECT(
      assert_equal(expected_error1, const_bias_factor.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(const_bias_factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
