/**
 * @file  testWrenchPlanarFactor.cpp
 * @brief test wrench planar factor
 * @Author: Yetong Zhang
 */
#include <WrenchPlanarFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace robot;

namespace example {
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_3x3);
Key wrench_key = Symbol('W', 1);
}  // namespace example

// Test wrench factor for stationary case with gravity
TEST(WrenchPlanarFactor, error_1) {
  // Create all factors
  Vector3 planar_axis;
  planar_axis << 1, 0, 0;
  WrenchPlanarFactor factor(example::wrench_key, example::cost_model, planar_axis);
  Vector wrench = (Vector(6) << 1, 2, 3, 4, 5, 6).finished();

  Vector3 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(wrench);
  expected_errors << 2, 3, 4;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
