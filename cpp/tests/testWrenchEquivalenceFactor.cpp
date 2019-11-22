/**
 * @file  testWrenchEquivalenceFactor.cpp
 * @brief test wrench factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <DHLink.h>
#include <WrenchEquivalenceFactor.h>

#include <cmath>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// R link example
DH_Link dh_r = DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3);
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);
Key twist_key = Symbol('V', 1), twist_accel_key = Symbol('T', 1),
    wrench_j_key = Symbol('W', 1), wrench_k_key = Symbol('W', 2),
    qKey = Symbol('q', 1), pKey = Symbol('p', 1);
}  // namespace example

/**
 * Test wrench factor for stationary case with gravity
 */
TEST(WrenchEquivalenceFactor, error_1) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  WrenchEquivalenceFactor factor(example::wrench_j_key, example::wrench_k_key,
                      example::qKey, example::cost_model, kMj, screw_axis);
  double q = 0;
  Vector wrench_j, wrench_k;
  wrench_j = (Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 19.6, 0, -9.8, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(wrench_j, wrench_k, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench factor for stationary case with gravity
 */
TEST(WrenchEquivalenceFactor, error_2) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  WrenchEquivalenceFactor factor(example::wrench_j_key, example::wrench_k_key,
                      example::qKey, example::cost_model, kMj, screw_axis);
  double q = -M_PI_2;
  Vector wrench_j, wrench_k;
  wrench_j = (Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 9.8, 9.8, 0, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(wrench_j, wrench_k, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
