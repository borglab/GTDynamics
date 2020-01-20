/**
 *  @file testActuatorJointFactor.cpp
 *  @test for pneumatic actuator facot
 *  @author Yetong
 **/
#include <PneumaticActuatorFactors.h>

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

TEST(PressureFactor, Factor) {
  Key start_t_key = Symbol('t', 0);
  Key current_t_key = Symbol('t', 1);
  Key init_p_key = Symbol('p', 0);
  Key p_key = Symbol('p', 1);

  const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
  const vector<double> coeffs {t0, c1, c2, c3};

  const double start_t = 0;
  const double current_t = 0.3;
  const double init_p = 240;
  const double p = 0;

  PressureFactor factor(start_t_key, current_t_key, init_p_key, p_key,
                        noiseModel::Isotropic::Sigma(1, 0.001), coeffs);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(start_t, current_t, init_p, p);
  expected_errors << 170.72941804811086;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(start_t_key, start_t);
  values.insert(current_t_key, current_t);
  values.insert(init_p_key, init_p);
  values.insert(p_key, p);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}


TEST(JointBalanceFactor, Factor) {
  Key delta_x_key = Symbol('x', 0);
  Key q_key = Symbol('q', 0);
  Key f_key = Symbol('f', 0);

  const double k = 1000;
  const double r = 0.1;
  const double qRest = 0.5;

  const double delta_x = 10;
  const double q = 1;
  const double f = 100;

  JointBalanceFactor factor(delta_x_key, q_key, f_key,
                                 noiseModel::Isotropic::Sigma(1, 0.001), k, r, qRest);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, q, f);
  expected_errors << 0.05;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(delta_x_key, delta_x);
  values.insert(q_key, q);
  values.insert(f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(PneumaticActuatorFactor, Factor) {
  Key delta_x_key = Symbol('x', 0);
  Key p_key = Symbol('p', 0);
  Key f_key = Symbol('f', 0);

  const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
               p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
               p12 = 0.0001081, p03 = -7.703e-07;
  const vector<double> coeffs {p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};

  const double delta_x = 1;
  const double p = 120;
  const double f = 0;

  PneumaticActuatorFactor factor(delta_x_key, p_key, f_key,
                                 noiseModel::Isotropic::Sigma(1, 0.001), coeffs);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << 193.17170160000003;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(delta_x_key, delta_x);
  values.insert(p_key, p);
  values.insert(f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
