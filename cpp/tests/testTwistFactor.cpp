/**
 * @file  testTwistFactor.cpp
 * @brief test twist factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <DhLink.h>
#include <TwistFactor.h>

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
DhLink dh_r =
    DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 10, 180);
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);
Key twist_i_key = Symbol('V', 1), twist_j_key = Symbol('V', 2),
    qKey = Symbol('q', 0), qVelKey = Symbol('j', 0);
}  // namespace example

// Test twist factor for stationary case
TEST(TwistFactor, error) {
  // Create all factors
  Pose3 jMi = Pose3(Rot3(), Point3(-1, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  TwistFactor factor(example::twist_i_key, example::twist_j_key, example::qKey,
                     example::qVelKey, example::cost_model, jMi, screw_axis);
  double q = M_PI / 4, qVel = 10;
  Vector twist_i, twist_j;
  twist_i = (Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_j = (Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(twist_i, twist_j, q, qVel);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::twist_i_key, twist_i);
  values.insert(example::twist_j_key, twist_j);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
