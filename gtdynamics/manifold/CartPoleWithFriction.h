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
#include <__functional/bind.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {
class CartPoleWithFriction {
public:
  double m = 1;
  double M = 1;
  double r = 1;
  double g = 10;
  double mu = 0.8;
  double tau_min = -20;
  double tau_max = 20;

  CartPoleWithFriction() {}

  double torque(const double q, const double a, OptionalJacobian<1, 1> H_q = {},
                OptionalJacobian<1, 1> H_a = {}) const {
    if (H_q)
      H_q->setConstant(-m * g * r * sin(q));
    if (H_a)
      H_a->setConstant(m * r * r);
    return m * a * r * r + m * g * r * cos(q);
  }

  double fx(const double q, const double v, const double a,
                       OptionalJacobian<1, 1> H_q = {},
                       OptionalJacobian<1, 1> H_v = {},
                       OptionalJacobian<1, 1> H_a = {}) const {
    double s = sin(q), c = cos(q);
    if (H_q)
      H_q->setConstant(m * v * v * r * s - m * a * r * c);
    if (H_v)
      H_v->setConstant(-2 * m * v * r * c);
    if (H_a)
      H_a->setConstant(-m * r * s);
    return -m * v * v * r * c - m * a * r * s;
  }

  double fy(const double q, const double v, const double a,
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

  // double friction_cone(const double q, const double v, const double a,
  //                      OptionalJacobian<1, 1> H_q = {},
  //                      OptionalJacobian<1, 1> H_v = {},
  //                      OptionalJacobian<1, 1> H_a = {}) const {
  //   double s = sin(q), c = cos(q);
  //   double fy = (M + m) * g - m * v * v * r * s + m * a * r * c;
  //   double fx = -m * v * v * r * c - m * a * r * s;
  //   double sign = fx > 0 ? 1 : -1;
  //   if (H_q)
  //     H_q->setConstant(mu * (-m * v * v * r * c - m * a * r * s) -
  //                      sign * (m * v * v * r * s - m * a * r * c));
  //   if (H_v)
  //     H_v->setConstant(mu * (-2 * m * v * r * s) - sign * (-2 * m * v * r * c));
  //   if (H_a)
  //     H_a->setConstant(mu * m * r * c - sign * (-m * r * s));
  //   return mu * fy - abs(fx);
  // }

  Double_ torqueExpr(const Double_ &q_expr, const Double_ &a_expr) {
    auto torque_function = [&](const double q, const double a,
                               OptionalJacobian<1, 1> H_q,
                               OptionalJacobian<1, 1> H_a) {
      if (H_q)
        H_q->setConstant(-m * g * r * sin(q));
      if (H_a)
        H_a->setConstant(m * r * r);
      return m * a * r * r + m * g * r * cos(q);
    };
    Double_ torque_expr(torque_function, q_expr, a_expr);
    return torque_expr;
  }

  std::pair<Double_, Double_> fcExprs(const Double_ &q_expr, const Double_ &v_expr,
                  const Double_ &a_expr) {
    auto fx_function = std::bind(&CartPoleWithFriction::fx, *this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
    auto fy_function = std::bind(&CartPoleWithFriction::fy, *this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
    Double_ fx_expr(fx_function, q_expr, v_expr, a_expr);
    Double_ fy_expr(fy_function, q_expr, v_expr, a_expr);
    Double_ fc1_expr = mu * fy_expr - fx_expr;
    Double_ fc2_expr = mu * fy_expr + fx_expr;
    return std::make_pair(fc1_expr, fc2_expr);
  }


  Double_ fcExpr(const Double_ &q_expr, const Double_ &v_expr,
                 const Double_ &a_expr) {
    auto friction_cone_function =
        [&](const double q, const double v, const double a,
            OptionalJacobian<1, 1> H_q, OptionalJacobian<1, 1> H_v,
            OptionalJacobian<1, 1> H_a) {
          double s = sin(q), c = cos(q);
          double fy = (M + m) * g - m * v * v * r * s + m * a * r * c;
          double fx = -m * v * v * r * c - m * a * r * s;
          double sign = fx > 0 ? 1 : -1;
          if (H_q)
            H_q->setConstant(mu * (-m * v * v * r * c - m * a * r * s) -
                             sign * (m * v * v * r * s - m * a * r * c));
          if (H_v)
            H_v->setConstant(mu * (-2 * m * v * r * s) -
                             sign * (-2 * m * v * r * c));
          if (H_a)
            H_a->setConstant(mu * m * r * c - sign * (-m * r * s));
          return mu * fy - abs(fx);
        };
    Double_ fc_expr(friction_cone_function, q_expr, v_expr, a_expr);
    return fc_expr;
  }
};

inline Key QKey(const int k) {
  return Symbol('q', k);
}

inline Key VKey(const int k) {
  return Symbol('v', k);
}

inline Key AKey(const int k) {
  return Symbol('a', k);
}

} // namespace gtsam
