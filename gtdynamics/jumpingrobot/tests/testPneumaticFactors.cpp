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

using gtdynamics::GassLawFactor, gtdynamics::MassFlowRateFactor,
    gtdynamics::ValveControlFactor;
using gtsam::Symbol, gtsam::Vector1, gtsam::Values, gtsam::Key,
    gtsam::assert_equal, gtsam::noiseModel::Isotropic;

TEST(GassLawFactor, Factor) {
  Key p_key = Symbol('p', 1);
  Key v_key = Symbol('v', 1);
  Key m_key = Symbol('m', 1);

  double c = 3;
  double p = 200;
  double v = 5e-5;
  double m = 3;

  GassLawFactor factor(p_key, v_key, m_key, Isotropic::Sigma(1, 0.001), c);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(p, v, m);
  expected_errors << 1;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(p_key, p);
  values.insert(v_key, v);
  values.insert(m_key, m);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

TEST(MassFlowRateFactor, Factor) {
  Key pm_key = Symbol('p', 0);
  Key pt_key = Symbol('p', 1);
  Key mdot_key = Symbol('m', 0);

  double pm = 100;
  double pt = 65.0 * 6.89476;
  double mdot = 1e-6;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;
  double mu = 1.8377e-5;
  double epsilon = 1e-5;
  double Rs = 287.0550;
  double T = 296.15;
  double k = 1. / (Rs * T);

  MassFlowRateFactor factor(pm_key, pt_key, mdot_key,
                            Isotropic::Sigma(1, 0.001), D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pm, pt, mdot);
  expected_errors << -3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(pm_key, pm);
  values.insert(pt_key, pt);
  values.insert(mdot_key, mdot);
  double diffDelta = 1e-10;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

TEST(MassFlowRateFactor, Negative) {
  Key pm_key = Symbol('p', 0);
  Key pt_key = Symbol('p', 1);
  Key mdot_key = Symbol('m', 0);

  double pm = 65.0 * 6.89476;
  double pt = 100;
  double mdot = -1e-6;
  double D = 0.1575 * 0.0254;
  double L = 74 * 0.0254;
  double mu = 1.8377e-5;
  double epsilon = 1e-5;
  double Rs = 287.0550;
  double T = 296.15;
  double k = 1. / (Rs * T);

  MassFlowRateFactor factor(pm_key, pt_key, mdot_key,
                            Isotropic::Sigma(1, 0.001), D, L, mu, epsilon, k);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pm, pt, mdot);
  expected_errors << 3.7545348620656125e-07;

  // EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(pm_key, pm);
  values.insert(pt_key, pt);
  values.insert(mdot_key, mdot);
  double diffDelta = 1e-10;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

TEST(ValveControlFactor, Factor) {
  Key t_key = Symbol('t', 0);
  Key to_key = Symbol('t', 1);
  Key tc_key = Symbol('t', 2);
  Key mdot_key = Symbol('m', 0);
  Key true_mdot_key = Symbol('m', 1);

  double t = 0.8;
  double to = 0.7;
  double tc = 1.2;
  double mdot = 2;
  double true_mdot = 1.5;
  double ct = 0.1;

  ValveControlFactor factor(t_key, to_key, tc_key, mdot_key, true_mdot_key,
                            Isotropic::Sigma(1, 0.001), ct);

  Vector1 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(t, to, tc, mdot, true_mdot);
  expected_errors << -0.07385526266417286;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-5));
  // Make sure linearization is correct
  Values values;
  values.insert(t_key, t);
  values.insert(to_key, to);
  values.insert(tc_key, tc);
  values.insert(true_mdot_key, true_mdot);
  values.insert(mdot_key, mdot);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-5);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
