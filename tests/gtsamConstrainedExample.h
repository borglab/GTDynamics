/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  gtsamConstrainedExample.h
 * @brief Simple constrained optimization example using GTSAM constraints.
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace constrained_example {

using namespace gtsam;

/// Exponential function e^x.
inline double exp_func(const double& x, gtsam::OptionalJacobian<1, 1> H1 = {}) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

/// Exponential expression e^x.
inline Double_ exp(const Double_& x) { return Double_(exp_func, x); }

/// Pow functor used for pow function.
class PowFunctor {
 private:
  double c_;

 public:
  explicit PowFunctor(const double& c) : c_(c) {}

  double operator()(const double& x,
                    gtsam::OptionalJacobian<1, 1> H1 = {}) const {
    if (H1) H1->setConstant(c_ * pow(x, c_ - 1));
    return pow(x, c_);
  }
};

/// Pow function.
inline Double_ pow(const Double_& x, const double& c) {
  PowFunctor pow_functor(c);
  return Double_(pow_functor, x);
}

/// Plus between Double expression and double.
inline Double_ operator+(const Double_& x, const double& d) {
  return x + Double_(d);
}

/// Negative sign operator.
inline Double_ operator-(const Double_& x) { return Double_(0.0) - x; }

/// Keys for creating expressions.
inline Symbol x1_key('x', 1);
inline Symbol x2_key('x', 2);
inline Double_ x1(x1_key), x2(x2_key);

}  // namespace constrained_example

namespace constrained_example1 {
using namespace constrained_example;

inline NonlinearFactorGraph Cost() {
  NonlinearFactorGraph graph;
  auto f1 = x1 + exp(-x2);
  auto f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));
  return graph;
}

inline NonlinearEqualityConstraints EqConstraints() {
  NonlinearEqualityConstraints constraints;
  auto h1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      h1, 0.0, Vector1(1.0));
  return constraints;
}

inline Values InitValues() {
  Values values;
  values.insert(x1_key, -0.2);
  values.insert(x2_key, -0.2);
  return values;
}

inline Values OptimalValues() {
  Values values;
  values.insert(x1_key, 0.0);
  values.insert(x2_key, 0.0);
  return values;
}

inline NonlinearFactorGraph costs = Cost();
inline NonlinearEqualityConstraints eqConstraints = EqConstraints();
inline NonlinearInequalityConstraints ineqConstraints;
inline Values init_values = InitValues();
inline ConstrainedOptProblem problem(costs, eqConstraints, ineqConstraints);
inline Values optimal_values = OptimalValues();

}  // namespace constrained_example1
