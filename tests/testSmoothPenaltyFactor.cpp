/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSmoothPenaltyFactor.cpp
 * @brief test smooth penalty factor.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/SmoothPenaltyFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

TEST(SmoothPenaltyFactor, negative) {
  Key x_key = 1;
  double min_val = -2;
  double max_val = 3;
  double b = 2;
  auto noise = noiseModel::Unit::Create(1);
  auto factor = SmoothPenaltyFactor(x_key, min_val, max_val, b, noise);

  // Check error and jacobian.
  {
    Values values1, values2, values3, values4, values5;
    values1.insert(x_key, -5.0);
    values2.insert(x_key, -3.0);
    values3.insert(x_key, 0.0);
    values4.insert(x_key, 4.0);
    values5.insert(x_key, 8.0);

    EXPECT(assert_equal(Vector1(2), factor->unwhitenedError(values1)));
    EXPECT(assert_equal(Vector1(0.25), factor->unwhitenedError(values2)));
    EXPECT(assert_equal(Vector1(0), factor->unwhitenedError(values3)));
    EXPECT(assert_equal(Vector1(0.25), factor->unwhitenedError(values4)));
    EXPECT(assert_equal(Vector1(4), factor->unwhitenedError(values5)));

    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values1, 1e-7, 1e-5);
    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values2, 1e-7, 1e-5);
    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values3, 1e-7, 1e-5);
    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values4, 1e-7, 1e-5);
    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values5, 1e-7, 1e-5);
  }

  // Check continuity and smoothness.
  {
    Values values1, values2;
    double delta = 1e-7;
    values1.insert(x_key, min_val - delta);
    values2.insert(x_key, min_val + delta);
    std::vector<Matrix> H1(1);
    std::vector<Matrix> H2(1);
    auto error1 = factor->unwhitenedError(values1, H1);
    auto error2 = factor->unwhitenedError(values2, H2);
    EXPECT(assert_equal(error1, error2, 1e-5));
    EXPECT(assert_equal(H1[0], H2[0], 1e-5));
  }
  {
    Values values1, values2;
    double delta = 1e-7;
    values1.insert(x_key, max_val - delta);
    values2.insert(x_key, max_val + delta);
    std::vector<Matrix> H1(1);
    std::vector<Matrix> H2(1);
    auto error1 = factor->unwhitenedError(values1, H1);
    auto error2 = factor->unwhitenedError(values2, H2);
    EXPECT(assert_equal(error1, error2, 1e-5));
    EXPECT(assert_equal(H1[0], H2[0], 1e-5));
  }
}



int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
