/**
 * @file  testTorqueFactor.cpp
 * @brief test torque factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <DhLink.h>
#include <TorqueFactor.h>

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
using namespace manipulator;

namespace example {
// R link example
DhLink dh_r = DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3,
                       -180, 10, 180);
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_1x1);
Key torque_key = Symbol('t', 1), wrench_key = Symbol('F', 1);
}  // namespace example

// Test Torque factor for stationary case
TEST(TorqueFactor, error) {
  // Create all factors
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  TorqueFactor factor(example::wrench_key, example::torque_key,
                      example::cost_model, screw_axis);
  double torque = 20;
  Vector wrench = (Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(wrench, torque);
  expected_errors = Vector1(0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::torque_key, torque);
  values.insert(example::wrench_key, wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
