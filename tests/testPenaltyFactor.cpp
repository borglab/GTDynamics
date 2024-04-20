/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPenaltyFactor.cpp
 * @brief test const bias factor.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PenaltyFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

TEST(PenaltyFactor, negative) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Base Factor.
  auto noise = noiseModel::Unit::Create(1);
  auto base_factor = std::make_shared<BetweenFactor<double>>(
      x1_key, x2_key, 1.0, noise);

  // Test constructor.
  PenaltyFactor penalty_factor(base_factor, false);

  // Check error and jacobian.
  {
    Values values;
    values.insert(x1_key, 0.1);
    values.insert(x2_key, 1.0);
    Vector expected_error1 = (Vector(1) << 0.0).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }

  {
    Values values;
    values.insert(x1_key, 1.0);
    values.insert(x2_key, 0.0);
    Vector expected_error1 = (Vector(1) << 0.0).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }
  
  {
    Values values;
    values.insert(x1_key, 0.0);
    values.insert(x2_key, 2.0);
    Vector expected_error1 = (Vector(1) << 1.0).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }
}


TEST(PenaltyFactor, positive) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Base Factor.
  auto noise = noiseModel::Unit::Create(1);
  auto base_factor = std::make_shared<BetweenFactor<double>>(
      x1_key, x2_key, 1.0, noise);

  // Test constructor.
  PenaltyFactor penalty_factor(base_factor, true);

  // Check error and jacobian.
  {
    Values values;
    values.insert(x1_key, 0.1);
    values.insert(x2_key, 1.0);
    Vector expected_error1 = (Vector(1) << 0.1).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }

  {
    Values values;
    values.insert(x1_key, 1.0);
    values.insert(x2_key, 0.0);
    Vector expected_error1 = (Vector(1) << 2.0).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }
  
  {
    Values values;
    values.insert(x1_key, 0.0);
    values.insert(x2_key, 2.0);
    Vector expected_error1 = (Vector(1) << 0.0).finished();
    EXPECT(assert_equal(expected_error1, penalty_factor.unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(penalty_factor, values, 1e-7, 1e-5);
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
