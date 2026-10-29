/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IECartPoleWithFriction.cpp
 * @brief Cart-pole trajectory optimization problem with friction cone limits.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;

/* ************************************************************************* */
double IECartPoleWithFriction::computeFx(const double q, const double v,
                                         const double a,
                                         OptionalJacobian<1, 1> H_q,
                                         OptionalJacobian<1, 1> H_v,
                                         OptionalJacobian<1, 1> H_a) const {
  double s = sin(q), c = cos(q);
  if (H_q)
    H_q->setConstant(mass_ * v * v * length_ * s - mass_ * a * length_ * c);
  if (H_v) H_v->setConstant(-2 * mass_ * length_ * v * c);
  if (H_a) H_a->setConstant(-mass_ * length_ * s);
  return -mass_ * v * v * length_ * c - mass_ * a * length_ * s;
}

/* ************************************************************************* */
double IECartPoleWithFriction::computeFy(const double q, const double v,
                                         const double a,
                                         OptionalJacobian<1, 1> H_q,
                                         OptionalJacobian<1, 1> H_v,
                                         OptionalJacobian<1, 1> H_a) const {
  double s = sin(q), c = cos(q);
  if (H_q)
    H_q->setConstant(-mass_ * v * v * length_ * c - mass_ * a * length_ * s);
  if (H_v) H_v->setConstant(-2 * mass_ * v * length_ * s);
  if (H_a) H_a->setConstant(mass_ * length_ * c);
  return (cart_mass_ + mass_) * gravity_ - mass_ * v * v * length_ * s +
         mass_ * a * length_ * c;
}

double IECartPoleWithFriction::computeTau(const double q, const double a,
                                          OptionalJacobian<1, 1> H_q,
                                          OptionalJacobian<1, 1> H_a) const {
  if (H_q) H_q->setConstant(-mass_ * gravity_ * length_ * sin(q));
  if (H_a) H_a->setConstant(mass_ * length_ * length_);
  return mass_ * length_ * length_ * a + mass_ * gravity_ * length_ * cos(q);
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceFxExpr(const Double_ &q_expr,
                                              const Double_ &v_expr,
                                              const Double_ &a_expr,
                                              const Double_ &fx_expr) const {
  auto compute_fx_function = [&](const double q, const double v, const double a,
                                 OptionalJacobian<1, 1> H_q = {},
                                 OptionalJacobian<1, 1> H_v = {},
                                 OptionalJacobian<1, 1> H_a = {}) {
    return computeFx(q, v, a, H_q, H_v, H_a);
  };
  Double_ compute_fx_expr(compute_fx_function, q_expr, v_expr, a_expr);
  return fx_expr - compute_fx_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceFyExpr(const Double_ &q_expr,
                                              const Double_ &v_expr,
                                              const Double_ &a_expr,
                                              const Double_ &fy_expr) const {
  auto compute_fy_function = [&](const double q, const double v, const double a,
                                 OptionalJacobian<1, 1> H_q = {},
                                 OptionalJacobian<1, 1> H_v = {},
                                 OptionalJacobian<1, 1> H_a = {}) {
    return computeFy(q, v, a, H_q, H_v, H_a);
  };
  Double_ compute_fy_expr(compute_fy_function, q_expr, v_expr, a_expr);
  return fy_expr - compute_fy_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::balanceRotExpr(const Double_ &q_expr,
                                               const Double_ &a_expr,
                                               const Double_ &tau_expr) const {
  auto compute_tau_function = [&](const double q, const double a,
                                  OptionalJacobian<1, 1> H_q = {},
                                  OptionalJacobian<1, 1> H_a = {}) {
    return computeTau(q, a, H_q, H_a);
  };
  Double_ compute_tau_expr(compute_tau_function, q_expr, a_expr);

  return tau_expr - compute_tau_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::frictionConeExpr1(
    const Double_ &fx_expr, const Double_ &fy_expr) const {
  return friction_coefficient_ * fy_expr - fx_expr;
}

/* ************************************************************************* */
Double_ IECartPoleWithFriction::frictionConeExpr2(
    const Double_ &fx_expr, const Double_ &fy_expr) const {
  return friction_coefficient_ * fy_expr + fx_expr;
}

/* ************************************************************************* */
NonlinearEqualityConstraints IECartPoleWithFriction::eConstraints(
    const int k) const {
  NonlinearEqualityConstraints constraints;
  Double_ q_expr(QKey(k));
  Double_ v_expr(VKey(k));
  Double_ a_expr(AKey(k));
  Double_ tau_expr(TauKey(k));
  Double_ fx_expr(FxKey(k));
  Double_ fy_expr(FyKey(k));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      balanceFxExpr(q_expr, v_expr, a_expr, fx_expr), 0.0, Vector1(1.0));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      balanceFyExpr(q_expr, v_expr, a_expr, fy_expr), 0.0, Vector1(1.0));
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      balanceRotExpr(q_expr, a_expr, tau_expr), 0.0, Vector1(1.0));
  return constraints;
}

/* ************************************************************************* */
NonlinearInequalityConstraints IECartPoleWithFriction::iConstraints(
    const int k) const {
  NonlinearInequalityConstraints constraints;
  Double_ fx_expr(FxKey(k));
  Double_ fy_expr(FyKey(k));
  constraints.push_back(ScalarExpressionInequalityConstraint::GeqZero(
      frictionConeExpr1(fx_expr, fy_expr), 1.0));
  constraints.push_back(ScalarExpressionInequalityConstraint::GeqZero(
      frictionConeExpr2(fx_expr, fy_expr), 1.0));
  if (include_torque_limits_) {
    Double_ torque_expr(TauKey(k));
    Double_ lower_limit_expr = torque_expr - Double_(torque_min_);
    Double_ upper_limit_expr = Double_(torque_max_) - torque_expr;
    constraints.push_back(
        ScalarExpressionInequalityConstraint::GeqZero(lower_limit_expr, 1.0));
    constraints.push_back(
        ScalarExpressionInequalityConstraint::GeqZero(upper_limit_expr, 1.0));
  }
  return constraints;
}

/* ************************************************************************* */
Values IECartPoleWithFriction::computeValues(const size_t &k, const double &q,
                                             const double &v,
                                             const double &a) const {
  Values values;
  double tau = computeTau(q, a);
  double fx = computeFx(q, v, a);
  double fy = computeFy(q, v, a);
  values.insertDouble(QKey(k), q);
  values.insertDouble(VKey(k), v);
  values.insertDouble(AKey(k), a);
  values.insertDouble(TauKey(k), tau);
  values.insertDouble(FxKey(k), fx);
  values.insertDouble(FyKey(k), fy);
  return values;
}

/* ************************************************************************* */
void IECartPoleWithFriction::PrintValues(const Values &values,
                                         const size_t num_steps) {
  std::cout << std::setw(12) << "q" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "tau" << std::setw(12) << "fx"
            << std::setw(12) << "fy"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.atDouble(QKey(k));
    double v = values.atDouble(VKey(k));
    double a = values.atDouble(AKey(k));
    double tau = values.atDouble(TauKey(k));
    double fx = values.atDouble(FxKey(k));
    double fy = values.atDouble(FyKey(k));
    std::cout << std::setprecision(5) << std::setw(12) << q << std::setw(12)
              << v << std::setw(12) << a << std::setw(12) << tau
              << std::setw(12) << fx << std::setw(12) << fy << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithFriction::PrintDelta(const VectorValues &values,
                                        const size_t num_steps) {
  std::cout << std::setw(12) << "q" << std::setw(12) << "v" << std::setw(12)
            << "a" << std::setw(12) << "tau" << std::setw(12) << "fx"
            << std::setw(12) << "fy"
            << "\n";
  for (size_t k = 0; k <= num_steps; k++) {
    double q = values.at(QKey(k))(0);
    double v = values.at(VKey(k))(0);
    double a = values.at(AKey(k))(0);
    double tau = values.at(TauKey(k))(0);
    double fx = values.at(FxKey(k))(0);
    double fy = values.at(FyKey(k))(0);
    std::cout << std::setw(12) << q << std::setw(12) << v << std::setw(12) << a
              << std::setw(12) << tau << std::setw(12) << fx << std::setw(12)
              << fy << "\n";
  }
}

/* ************************************************************************* */
void IECartPoleWithFriction::ExportValues(const Values &values,
                                          const size_t num_steps,
                                          const std::string &file_path) {
  std::ofstream file;
  file.open(file_path);
  file << "time,q,v,a,tau,fx,fy\n";
  double dt = 0.05;
  for (int k = 0; k <= num_steps; k++) {
    double t = dt * k;
    double q = values.atDouble(QKey(k));
    double v = values.atDouble(VKey(k));
    double a = values.atDouble(AKey(k));
    double tau = values.atDouble(TauKey(k));
    double fx = values.atDouble(FxKey(k));
    double fy = values.atDouble(FyKey(k));
    file << t << "," << q << "," << v << "," << a << "," << tau << "," << fx
         << "," << fy << "\n";
  }
  file.close();
}

/* ************************************************************************* */
void IECartPoleWithFriction::ExportVector(const VectorValues &values,
                                          const size_t num_steps,
                                          const std::string &file_path) {
  std::ofstream file;
  file.open(file_path);
  for (int k = 0; k <= num_steps; k++) {
    double q = values.at(QKey(k))(0);
    double v = values.at(VKey(k))(0);
    double a = values.at(AKey(k))(0);
    double tau = values.at(TauKey(k))(0);
    double fx = values.at(FxKey(k))(0);
    double fy = values.at(FyKey(k))(0);
    file << q << " " << v << " " << a << " " << tau << " " << fx << " " << fy
         << "\n";
  }
  file << "\n";
  file.close();
}

}  // namespace gtdynamics
