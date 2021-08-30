/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file testPneumaticFactors.cpp
 *  @brief Tests for pneumatic factors.
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

#include "gtdynamics/jumpingrobot/factors/PneumaticFactors.h"

using gtdynamics::GasLawFactor, gtdynamics::MassFlowRateFactor,
    gtdynamics::ValveControlFactor;
using gtsam::Symbol, gtsam::Vector1, gtsam::Values, gtsam::Key,
    gtsam::assert_equal, gtsam::noiseModel::Isotropic;

namespace example {
auto cost_model = Isotropic::Sigma(1, 0.001);
gtsam::Symbol p_key('p', 0), v_key('v', 0), m_key('m', 0), t_key('t', 0), to_key('t', 1), tc_key('t', 2), mdot_key('m', 1), true_mdot_key('m',2), pa_key('p', 1), ps_key('p', 2);
}  // namespace example

TEST(GasLawFactor, Factor) {
  double c = 3;
  double p = 200;
  double v = 5e-5;
  double m = 3;

  GasLawFactor factor(example::p_key, example::v_key, example::m_key, example::cost_model, c);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(p, v, m);
  expected_errors << 1;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::p_key, p);
  values.insert(example::v_key, v);
  values.insert(example::m_key, m);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

TEST(MassFlowRateFactor, Factor) {
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

  MassFlowRateFactor factor(example::pa_key, example::ps_key, example::mdot_key,
                            Isotropic::Sigma(1, 0.001), D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pa, ps, mdot);
  expected_errors << -3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::pa_key, pa);
  values.insert(example::ps_key, ps);
  values.insert(example::mdot_key, mdot);
  double diffDelta = 1e-10;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-2);
}

TEST(MassFlowRateFactor, Negative) {
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

  MassFlowRateFactor factor(example::pa_key, example::ps_key, example::mdot_key,
                            Isotropic::Sigma(1, 0.001), D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pa, ps, mdot);
  expected_errors << 3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::pa_key, pa);
  values.insert(example::ps_key, ps);
  values.insert(example::mdot_key, mdot);
  double diffDelta = 1e-13;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(ValveControlFactor, Factor) {
  double t = 0.8;
  double to = 0.7;
  double tc = 1.2;
  double mdot = 2;
  double true_mdot = 1.5;
  double ct = 0.1;

  ValveControlFactor factor(example::t_key, example::to_key, example::tc_key, example::mdot_key, example::true_mdot_key,
                            example::cost_model, ct);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(t, to, tc, mdot, true_mdot);
  expected_errors << -0.07385526266417286;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(example::t_key, t);
  values.insert(example::to_key, to);
  values.insert(example::tc_key, tc);
  values.insert(example::true_mdot_key, true_mdot);
  values.insert(example::mdot_key, mdot);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
