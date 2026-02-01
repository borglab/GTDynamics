/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  nithya_yetong00_constrainedopt_benchmark.cpp
 * @brief Benchmark penalty method optimizer and augmented lagrangian optimizer
 * on a toy example, and output intermediate results to file.
 * @author: Nithya Jayakumar
 * @author: Yetong Zhang
 */

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <fstream>
#include <iostream>

using namespace gtsam;

namespace {

/// Exponential function e^x.
double exp_func(const double& x, gtsam::OptionalJacobian<1, 1> H1 = {}) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

/// Exponential expression e^x.
Double_ exp(const Double_& x) { return Double_(exp_func, x); }

/// Pow functor used for pow function.
class PowFunctor {
 private:
  double c_;

 public:
  PowFunctor(const double& c) : c_(c) {}

  double operator()(const double& x,
                    gtsam::OptionalJacobian<1, 1> H1 = {}) const {
    if (H1) H1->setConstant(c_ * pow(x, c_ - 1));
    return pow(x, c_);
  }
};

/// Pow function.
Double_ pow(const Double_& x, const double& c) {
  PowFunctor pow_functor(c);
  return Double_(pow_functor, x);
}

/// Plus between Double expression and double.
Double_ operator+(const Double_& x, const double& d) {
  return x + Double_(d);
}

/// Negative sign operator.
Double_ operator-(const Double_& x) { return Double_(0.0) - x; }

Symbol x1_key('x', 1);
Symbol x2_key('x', 2);
Double_ x1(x1_key), x2(x2_key);

}  // namespace

int main(int argc, char** argv) {
  /// Create a constrained optimization problem with 2 cost factors and 1
  /// constraint.
  NonlinearFactorGraph graph;
  gtsam::Double_ f1 = x1 + exp(-x2);
  gtsam::Double_ f2 = pow(x1, 2.0) + 2.0 * x2 + 1.0;
  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  graph.add(ExpressionFactor<double>(cost_noise, 0., f1));
  graph.add(ExpressionFactor<double>(cost_noise, 0., f2));

  NonlinearEqualityConstraints constraints;
  double sigma = 1.0;
  gtsam::Double_ g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      g1, 0.0, Vector1(sigma));

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, -0.2);
  init_values.insert(x2_key, -0.2);

  auto problem = ConstrainedOptProblem::EqConstrainedOptProblem(graph, constraints);

  /// Solve the constraint problem with Penalty Method optimizer.
  auto penalty_params = std::make_shared<gtsam::PenaltyOptimizerParams>();
  gtsam::PenaltyOptimizer penalty_optimizer(problem, init_values, penalty_params);
  Values penalty_results = penalty_optimizer.optimize();

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  auto augl_params = std::make_shared<gtsam::AugmentedLagrangianParams>();
  gtsam::AugmentedLagrangianOptimizer augl_optimizer(problem, init_values,
                                                     augl_params);
  Values augl_results = augl_optimizer.optimize();

  /// Function to evaluate constraint violation.
  auto evaluate_constraint = [&constraints](const gtsam::Values& values) {
    return constraints.violationNorm(values);
  };

  /// Function to evaluate cost.
  auto evaluate_cost = [&graph](const gtsam::Values& values) {
    double cost = graph.error(values);
    return cost;
  };

  /// Write results to files for plotting.
  std::cout << "Penalty result: cost=" << evaluate_cost(penalty_results)
            << " constraint=" << evaluate_constraint(penalty_results) << "\n";
  std::cout << "Augmented Lagrangian result: cost="
            << evaluate_cost(augl_results)
            << " constraint=" << evaluate_constraint(augl_results) << "\n";
  return 0;
}
