/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file testSystemIdentificationFactors.cpp
 *  @brief Tests for system identification factors.
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

#include "gtdynamics/jumpingrobot/factors/SystemIdentificationFactors.h"

using gtdynamics::MassFlowRateFactorId, gtdynamics::ForceBalanceFactorId,
    gtdynamics::JointTorqueFactorId, gtdynamics::ActuatorVolumeFactorId;
using gtsam::Symbol, gtsam::Vector1, gtsam::Values, gtsam::Key,
    gtsam::assert_equal, gtsam::noiseModel::Isotropic;

namespace example {
auto cost_model = Isotropic::Sigma(1, 0.001);
gtsam::Symbol p_key('p', 0), v_key('v', 0), m_key('m', 0), t_key('t', 0),
    to_key('t', 1), tc_key('t', 2), mdot_key('m', 1), true_mdot_key('m', 2),
    pa_key('p', 1), ps_key('p', 2), f_key('f', 0), q_key('q', 0),
    delta_x_key('x', 0), torque_key('T', 0), l_key('l', 0), d_tube_key('d', 0),
    k_key('k', 0), b_key('b', 0);
}  // namespace example

TEST(MassFlowRateFactorId, Positive) {
  double pa = 100;
  double ps = 65.0 * 6.89476;
  double mdot = 1e-6;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;
  double mu = 1.8377e-5;
  double epsilon = 1e-5;
  double Rs = 287.0550;
  double T = 296.15;
  double k = 1. / (Rs * T);

  MassFlowRateFactorId factor(example::pa_key, example::ps_key,
                              example::mdot_key, example::d_tube_key,
                              example::cost_model, D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pa, ps, mdot, D);
  expected_errors << -3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::pa_key, pa);
  values.insert(example::ps_key, ps);
  values.insert(example::mdot_key, mdot);
  values.insert(example::d_tube_key, D);
  double diffDelta = 1e-10;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-2);
}

TEST(MassFlowRateFactorId, Negative) {
  double pa = 65.0 * 6.89476;
  double ps = 100;
  double mdot = -1e-6;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;
  double mu = 1.8377e-5;
  double epsilon = 1e-5;
  double Rs = 287.0550;
  double T = 296.15;
  double k = 1. / (Rs * T);

  MassFlowRateFactorId factor(example::pa_key, example::ps_key,
                              example::mdot_key, example::d_tube_key,
                              example::cost_model, D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pa, ps, mdot, D);
  expected_errors << 3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::pa_key, pa);
  values.insert(example::ps_key, ps);
  values.insert(example::mdot_key, mdot);
  values.insert(example::d_tube_key, D);
  double diffDelta = 1e-11;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-4);
}

/** Test the force balance with contract configuration of the actuator. */
TEST(ForceBalanceFactorId, Contract) {
  double kt = 8200;
  double r = 0.02;
  double q_rest = 0.5;
  bool positive = false;
  ForceBalanceFactorId factor(example::delta_x_key, example::q_key,
                              example::f_key, example::k_key,
                              example::cost_model, r, q_rest, positive);

  double delta_x = 0.4;
  double q = 0.8;
  double f = 10;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(delta_x, q, f, kt);
  expected_errors << 72;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::q_key, q);
  values.insert(example::f_key, f);
  values.insert(example::k_key, kt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/** Test the force balance with expand configuration of the actuator. */
TEST(ForceBalanceFactorId, Expand) {
  double kt = 8200;
  double r = 0.02;
  double q_rest = 0.5;
  bool positive = true;
  ForceBalanceFactorId factor(example::delta_x_key, example::q_key,
                              example::f_key, example::k_key,
                              example::cost_model, r, q_rest, positive);

  double delta_x = 0.4;
  double q = 0.8;
  double f = 10;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(delta_x, q, f, kt);
  expected_errors << -26.4;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::delta_x_key, delta_x);
  values.insert(example::q_key, q);
  values.insert(example::f_key, f);
  values.insert(example::k_key, kt);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/** Test the joint torque factor with expand configuration of the actuator,
 * and anatagnostic spring inactive. */
TEST(JointTorqueFactor, ExpandInactive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = false;

  JointTorqueFactorId factor(example::q_key, example::v_key, example::f_key,
                             example::torque_key, example::b_key,
                             example::cost_model, q_limit, ka, r, positive);

  double q = 0.8;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque, b);
  expected_errors << -1.26;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  values.insert(example::b_key, b);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointTorqueFactorId, ContractActive) {
  double q_limit = 0.4;
  double ka = 5;
  double r = 0.02;
  double b = 0.6;
  bool positive = true;

  JointTorqueFactorId factor(example::q_key, example::v_key, example::f_key,
                             example::torque_key, example::b_key,
                             example::cost_model, q_limit, ka, r, positive);

  double q = 0.8;
  double v = 0.1;
  double f = 10;
  double torque = 1;

  Vector1 actual_errors, expected_errors;
  actual_errors = factor.evaluateError(q, v, f, torque, b);
  expected_errors << -2.86;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-3));

  Values values;
  values.insert(example::q_key, q);
  values.insert(example::v_key, v);
  values.insert(example::f_key, f);
  values.insert(example::torque_key, torque);
  values.insert(example::b_key, b);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/** Check if actuator volume computaiton is correct */
TEST(ActuatorVolumeFactorId, Factor) {
  double l = 10;
  double v = 0.001;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;

  ActuatorVolumeFactorId factor(example::v_key, example::l_key,
                                example::d_tube_key, example::cost_model, L);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(v, l, D);
  expected_errors << -0.000816944;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::v_key, v);
  values.insert(example::l_key, l);
  values.insert(example::d_tube_key, D);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
