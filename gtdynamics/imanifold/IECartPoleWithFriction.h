/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IneqConstraintManifold.h
 * @brief Manifold with boundary/corners formulated by only inequality
 * constraints
 * @author: Yetong Zhang
 */

#pragma once
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

inline Key QKey(const int k) { return Symbol('q', k); }

inline Key VKey(const int k) { return Symbol('v', k); }

inline Key AKey(const int k) { return Symbol('a', k); }

inline Key TauKey(const int k) { return Symbol('t', k); }

inline Key FxKey(const int k) { return Symbol('x', k); }

inline Key FyKey(const int k) { return Symbol('y', k); }

inline void PrintValues(const Values& values, const size_t num_steps) {
  std::cout << std::setw(10) << "q" << std::setw(10)  << "v" << std::setw(10) << "a" << std::setw(10) << "tau" << std::setw(10) << "fx" << std::setw(10) << "fy" << "\n";
  for (size_t k=0; k<=num_steps; k++) {
    double q = values.atDouble(QKey(k));
    double v = values.atDouble(VKey(k));
    double a = values.atDouble(AKey(k));
    double tau = values.atDouble(TauKey(k));
    double fx = values.atDouble(FxKey(k));
    double fy = values.atDouble(FyKey(k));
    std::cout << std::setprecision(3)<< std::setw(10) << q << std::setw(10)  << v << std::setw(10) << a << std::setw(10) << tau << std::setw(10) << fx << std::setw(10) << fy << "\n";
  }
}

inline void PrintDelta(const VectorValues& values, const size_t num_steps) {
  std::cout << std::setw(10) << "q" << std::setw(10)  << "v" << std::setw(10) << "a" << std::setw(10) << "tau" << std::setw(10) << "fx" << std::setw(10) << "fy" << "\n";
  for (size_t k=0; k<=num_steps; k++) {
    double q = values.at(QKey(k))(0);
    double v = values.at(VKey(k))(0);
    double a = values.at(AKey(k))(0);
    double tau = values.at(TauKey(k))(0);
    double fx = values.at(FxKey(k))(0);
    double fy = values.at(FyKey(k))(0);
    std::cout <<std::setw(10) << q << std::setw(10)  << v << std::setw(10) << a << std::setw(10) << tau << std::setw(10) << fx << std::setw(10) << fy << "\n";
  }
}

class IECartPoleWithFriction {
public:
  double m = 1;
  double M = 1;
  double r = 1;
  double g = 10;
  double mu = 0.8;
  double tau_min = -20;
  double tau_max = 20;

  IECartPoleWithFriction() {}

  double computeFx(const double q, const double v, const double a,
                   OptionalJacobian<1, 1> H_q = {},
                   OptionalJacobian<1, 1> H_v = {},
                   OptionalJacobian<1, 1> H_a = {}) const {
    double s = sin(q), c = cos(q);
    if (H_q)
      H_q->setConstant(m * v * v * r * s - m * a * r * c);
    if (H_v)
      H_v->setConstant(-2 * m * r * v * c);
    if (H_a)
      H_a->setConstant(-m * r * s);
    return -m * v * v * r * c - m * a * r * s;
  }

  double computeFy(const double q, const double v, const double a,
                   OptionalJacobian<1, 1> H_q = {},
                   OptionalJacobian<1, 1> H_v = {},
                   OptionalJacobian<1, 1> H_a = {}) const {
    double s = sin(q), c = cos(q);
    if (H_q)
      H_q->setConstant(-m * v * v * r * c - m * a * r * s);
    if (H_v)
      H_v->setConstant(-2 * m * v * r * s);
    if (H_a)
      H_a->setConstant(m * r * c);
    return (M + m) * g - m * v * v * r * s + m * a * r * c;
  }

  double computeTau(const double a) const {
    return m * r * r * a;
  }

  Double_ balanceFxExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fx_expr) const {
    auto compute_fx_function =
        [&](const double q, const double v, const double a,
            OptionalJacobian<1, 1> H_q = {}, OptionalJacobian<1, 1> H_v = {},
            OptionalJacobian<1, 1> H_a = {}) {
              return computeFx(q, v, a, H_q, H_v, H_a);
        };
    Double_ compute_fx_expr(compute_fx_function, q_expr, v_expr, a_expr);
    return fx_expr - compute_fx_expr;
  }

  Double_ balanceFyExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fy_expr) const {
    auto compute_fy_function =
        [&](const double q, const double v, const double a,
            OptionalJacobian<1, 1> H_q = {}, OptionalJacobian<1, 1> H_v = {},
            OptionalJacobian<1, 1> H_a = {}) {
          return computeFy(q, v, a, H_q, H_v, H_a);
        };
    Double_ compute_fy_expr(compute_fy_function, q_expr, v_expr, a_expr);
    return fy_expr - compute_fy_expr;
  }

  Double_ balanceRotExpr(const Double_ &a_expr, const Double_ &tau_expr) const {
    return tau_expr - m * r * r * a_expr;
  }

  Double_ frictionConeExpr1(const Double_ &fx_expr,
                            const Double_ &fy_expr) const {
    return mu * fy_expr - fx_expr;
  }

  Double_ frictionConeExpr2(const Double_ &fx_expr,
                            const Double_ &fy_expr) const {
    return mu * fy_expr + fx_expr;
  }

  gtdynamics::EqualityConstraints eConstraints(const int k) const {
    gtdynamics::EqualityConstraints constraints;
    Double_ q_expr(QKey(k));
    Double_ v_expr(VKey(k));
    Double_ a_expr(AKey(k));
    Double_ tau_expr(TauKey(k));
    Double_ fx_expr(FxKey(k));
    Double_ fy_expr(FyKey(k));
    constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
        balanceFxExpr(q_expr, v_expr, a_expr, fx_expr), 1.0);
    constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
        balanceFyExpr(q_expr, v_expr, a_expr, fy_expr), 1.0);
    constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(
        balanceRotExpr(a_expr, tau_expr), 1.0);
    return constraints;
  }

  gtdynamics::InequalityConstraints iConstraints(const int k) const {
    gtdynamics::InequalityConstraints constraints;
    Double_ fx_expr(FxKey(k));
    Double_ fy_expr(FyKey(k));
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        frictionConeExpr1(fx_expr, fy_expr), 1.0);
    constraints.emplace_shared<gtdynamics::DoubleExpressionInequality>(
        frictionConeExpr2(fx_expr, fy_expr), 1.0);
    return constraints;
  }

  Values computeValues(const size_t &k, const double &q, const double &v,
                       const double &a) const {
    Values values;
    double tau = computeTau(a);
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
};

} // namespace gtsam
