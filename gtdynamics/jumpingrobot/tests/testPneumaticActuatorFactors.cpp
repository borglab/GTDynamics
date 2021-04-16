/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file testActuatorJointFactor.cpp
 *  @brief Tests for pneumatic actuator factors.
 *  @author Yetong Zhang
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

#include "gtdynamics/jumpingrobot/factors/PneumaticActuatorFactors.h"

using gtdynamics::ForceBalanceFactor, gtdynamics::JointTorqueFactor, gtdynamics::ActuatorVolumeFactor,
    gtdynamics::SmoothActuatorFactor, gtdynamics::ClippingActuatorFactor;
using gtsam::Symbol, gtsam::Vector1, gtsam::Values, gtsam::Key,
    gtsam::assert_equal, gtsam::noiseModel::Isotropic;

namespace example {
  auto cost_model = Isotropic::Sigma(1, 0.001);
  gtsam::Symbol q_key('q', 0), v_key('v', 0), f_key('f', 0), torque_key('T', 0),
    l_key('l', 0), p_key('p', 0), delta_x_key('x', 0);
}  // namespace example


TEST(ForceBalanceFactor, Contract) {
  double kt = 8200;
  double r = 0.02;
  double q_rest = 0.5;
  bool positive = false;
  ForceBalanceFactor factor(example::delta_x_key, example::q_key, example::f_key, example::cost_model, kt, r, q_rest, positive);

  double delta_x = 0.4;
  double q = 0.8;
  double f = 10;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(delta_x, q, f);
  expected_errors << 72;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::q_key, q);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ForceBalanceFactor, Expand) {
  double kt = 8200;
  double r = 0.02;
  double q_rest = 0.5;
  bool positive = true;
  ForceBalanceFactor factor(example::delta_x_key, example::q_key, example::f_key, example::cost_model, kt, r, q_rest, positive);

  double delta_x = 0.4;
  double q = 0.8;
  double f = 10;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(delta_x, q, f);
  expected_errors << -26.4;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::q_key, q);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointTorqueFactor, ExpandInactive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = false;

  JointTorqueFactor factor(example::q_key, example::v_key, example::f_key,
                           example::torque_key, example::cost_model, q_limit,
                           ka, r, b, positive);

  double q = 0.8;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque);
  expected_errors << -1.26;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointTorqueFactor, ContractActive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = true;

  JointTorqueFactor factor(example::q_key, example::v_key, example::f_key,
                           example::torque_key, example::cost_model, q_limit,
                           ka, r, b, positive);

  double q = 0.8;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque);
  expected_errors << -2.86;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointTorqueFactor, ExpandActive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = false;

  JointTorqueFactor factor(example::q_key, example::v_key, example::f_key,
                           example::torque_key, example::cost_model, q_limit,
                           ka, r, b, positive);
  double q = 0.0;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque);
  expected_errors << 0.74;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointTorqueFactor, contractInactive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = true;

  JointTorqueFactor factor(example::q_key, example::v_key, example::f_key,
                           example::torque_key, example::cost_model, q_limit,
                           ka, r, b, positive);

  double q = 0.0;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque);
  expected_errors << -0.86;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ActuatorVolumeFactor, Factor) {
  double l = 10;
  double v = 0.001;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;

  ActuatorVolumeFactor factor(example::v_key, example::l_key,
                              example::cost_model, D, L);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(v, l);
  expected_errors << -0.000816944;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::v_key, v);
  values.insert(example::l_key, l);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

TEST(SmoothActuatorFactor, negative_contraction_zero) {
  const double delta_x = 2;
  const double p = 800;
  const double f = 0;

  SmoothActuatorFactor factor(example::delta_x_key, example::p_key,
                              example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-1));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

//// following tests are deprecated
TEST(ClippingActuatorFactor, Factor) {
  const double delta_x = 1;
  const double p = 120;
  const double f = 0;

  ClippingActuatorFactor factor(example::delta_x_key, example::p_key,
                                example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << 193.17170160000003;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ClippingActuatorFactor, negative_force) {
  const double delta_x = 10;
  const double p = 240;
  const double f = 100;

  ClippingActuatorFactor factor(example::delta_x_key, example::p_key,
                                example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << -100;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ClippingActuatorFactor, zero_region) {
  const double delta_x = 13;
  const double p = 200;
  const double f = 100;

  ClippingActuatorFactor factor(example::delta_x_key, example::p_key,
                                example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << -100;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ClippingActuatorFactor, negative_contraction) {
  const double delta_x = -1.9;
  const double p = 345;
  const double f = 1059.28;

  ClippingActuatorFactor factor(example::delta_x_key, example::p_key,
                                example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-1));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ClippingActuatorFactor, negative_contraction_zero) {
  const double delta_x = -2;
  const double p = 1;
  const double f = 400;

  ClippingActuatorFactor factor(example::delta_x_key, example::p_key,
                                example::f_key, example::cost_model);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(delta_x, p, f);
  expected_errors << 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-1));
  // Make sure linearization is correct
  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::p_key, p);
  values.insert(example::f_key, f);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
