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
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IERetractor.h>
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
                   OptionalJacobian<1, 1> H_a = {}) const;

  double computeFy(const double q, const double v, const double a,
                   OptionalJacobian<1, 1> H_q = {},
                   OptionalJacobian<1, 1> H_v = {},
                   OptionalJacobian<1, 1> H_a = {}) const;

  double computeTau(const double a) const { return m * r * r * a; }

  Double_ balanceFxExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fx_expr) const;

  Double_ balanceFyExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fy_expr) const;

  Double_ balanceRotExpr(const Double_ &a_expr, const Double_ &tau_expr) const;

  Double_ frictionConeExpr1(const Double_ &fx_expr,
                            const Double_ &fy_expr) const;

  Double_ frictionConeExpr2(const Double_ &fx_expr,
                            const Double_ &fy_expr) const;

  gtdynamics::EqualityConstraints eConstraints(const int k) const;
  gtdynamics::InequalityConstraints iConstraints(const int k) const;

  Values computeValues(const size_t &k, const double &q, const double &v,
                       const double &a) const;

  Values defaultValues(const size_t &k) const {return computeValues(k, 0, 0, 0); }

  static void PrintValues(const Values &values, const size_t num_steps);

  static void PrintDelta(const VectorValues &values, const size_t num_steps);
};

class CartPoleWithFrictionRetractor : public IERetractor {
protected:
  const IECartPoleWithFriction &cp_;

public:
  CartPoleWithFrictionRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;

  IEConstraintManifold retract1(const IEConstraintManifold *manifold,
                                const VectorValues &delta) const;
};

class CPBarrierRetractor : public IERetractor {
protected:
  const IECartPoleWithFriction &cp_;

public:
  CPBarrierRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;
};

} // namespace gtsam
