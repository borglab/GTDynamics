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
};

} // namespace gtsam
