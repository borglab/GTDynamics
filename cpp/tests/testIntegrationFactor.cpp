/**
 *  @file testIntegrationFactor.cpp
 *  @test for integration factor for joint angle, joint
 *   velocity, and joint acceleration
 *  @author Yetong
 **/
#include <IntegrationFactor.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace robot;

TEST(IntegrationFactor, Factor) {
  const double delta_t = 0.1;
  Key q1_key = Symbol('x', 1), q2_key = Symbol('x', 2);
  Key qVel1_key = Symbol('v', 1), qVel2_key = Symbol('v', 2);
  Key qAccel1_key = Symbol('a', 1);
  IntegrationFactor factor(q1_key, qVel1_key, qAccel1_key, q2_key, qVel2_key,
                           noiseModel::Isotropic::Sigma(2, 0.001), delta_t);

  double q1 = 1, qVel1 = 1, qAccel1 = 1, q2 = 1, qVel2 = 1;
  Vector2 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(q1, qVel1, qAccel1, q2, qVel2);
  expected_errors << 0.105, 0.1;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(q1_key, q1);
  values.insert(qVel1_key, qVel1);
  values.insert(qAccel1_key, qAccel1);
  values.insert(q2_key, q2);
  values.insert(qVel2_key, qVel2);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(SoftIntegrationFactor, Factor) {
  Key q1_key = Symbol('x', 1), q2_key = Symbol('x', 2);
  Key qVel1_key = Symbol('v', 1), qVel2_key = Symbol('v', 2);
  Key qAccel1_key = Symbol('a', 1), dt_key = Symbol('t', 0);
  SoftIntegrationFactor factor(q1_key, qVel1_key, qAccel1_key, q2_key, qVel2_key, dt_key,
                           noiseModel::Isotropic::Sigma(2, 0.001));

  double q1 = 1, qVel1 = 1, qAccel1 = 1, q2 = 1, qVel2 = 1, dt=0.1;
  Vector2 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(q1, qVel1, qAccel1, q2, qVel2, dt);
  expected_errors << 0.105, 0.1;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(q1_key, q1);
  values.insert(qVel1_key, qVel1);
  values.insert(qAccel1_key, qAccel1);
  values.insert(q2_key, q2);
  values.insert(qVel2_key, qVel2);
  values.insert(dt_key, dt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
