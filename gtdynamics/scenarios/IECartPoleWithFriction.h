/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IECartPoleWithFriction.h
 * @brief Cart-pole trajectory optimization problem with friction cone limits.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/nonlinear/expressions.h>

#include <string>

namespace gtdynamics {
using namespace gtsam;

/** Key for the pole angle at timestep k. */
inline Key QKey(const int k) { return Symbol('q', k); }

/** Key for the pole angular velocity at timestep k. */
inline Key VKey(const int k) { return Symbol('v', k); }

/** Key for the pole angular acceleration at timestep k. */
inline Key AKey(const int k) { return Symbol('a', k); }

/** Key for the pole torque at timestep k. */
inline Key TauKey(const int k) { return Symbol('t', k); }

/** Key for the horizontal contact force at timestep k. */
inline Key FxKey(const int k) { return Symbol('x', k); }

/** Key for the vertical contact force at timestep k. */
inline Key FyKey(const int k) { return Symbol('y', k); }

/** Cart-pole problem with equality dynamics and friction cone constraints. */
class IECartPoleWithFriction {
 public:
  /// Pole mass.
  double mass_ = 1;

  /// Cart mass.
  double cart_mass_ = 1;

  /// Pole length.
  double length_ = 1;

  /// Gravitational acceleration magnitude.
  double gravity_ = 10;

  /// Contact friction coefficient.
  double friction_coefficient_ = 0.8;

  /// Lower torque limit.
  double torque_min_ = -25;

  /// Upper torque limit.
  double torque_max_ = 25;

  /// Whether torque limits are included as inequality constraints.
  bool include_torque_limits_ = false;

  /** Constructor. */
  IECartPoleWithFriction() {}

  /** Compute horizontal contact force from pole state and acceleration. */
  double computeFx(const double q, const double v, const double a,
                   OptionalJacobian<1, 1> H_q = {},
                   OptionalJacobian<1, 1> H_v = {},
                   OptionalJacobian<1, 1> H_a = {}) const;

  /** Compute vertical contact force from pole state and acceleration. */
  double computeFy(const double q, const double v, const double a,
                   OptionalJacobian<1, 1> H_q = {},
                   OptionalJacobian<1, 1> H_v = {},
                   OptionalJacobian<1, 1> H_a = {}) const;

  /** Compute pole torque from pole angle and acceleration. */
  double computeTau(const double q, const double a,
                    OptionalJacobian<1, 1> H_q = {},
                    OptionalJacobian<1, 1> H_a = {}) const;

  /** Return an expression for horizontal force balance. */
  Double_ balanceFxExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fx_expr) const;

  /** Return an expression for vertical force balance. */
  Double_ balanceFyExpr(const Double_ &q_expr, const Double_ &v_expr,
                        const Double_ &a_expr, const Double_ &fy_expr) const;

  /** Return an expression for rotational balance. */
  Double_ balanceRotExpr(const Double_ &q_expr, const Double_ &a_expr,
                         const Double_ &tau_expr) const;

  /** Return the first friction cone inequality expression. */
  Double_ frictionConeExpr1(const Double_ &fx_expr,
                            const Double_ &fy_expr) const;

  /** Return the second friction cone inequality expression. */
  Double_ frictionConeExpr2(const Double_ &fx_expr,
                            const Double_ &fy_expr) const;

  /** Return equality constraints for a single time step. */
  NonlinearEqualityConstraints eConstraints(const int k) const;

  /** Return friction cone and optional torque limit constraints. */
  NonlinearInequalityConstraints iConstraints(const int k) const;

  /** Compute a full dynamics-consistent state from pole coordinates. */
  Values computeValues(const size_t &k, const double &q, const double &v,
                       const double &a) const;

  /** Return default rest values for a single time step. */
  Values defaultValues(const size_t &k) const {
    return computeValues(k, 0, 0, 0);
  }

  /** Print trajectory values in a compact table. */
  static void PrintValues(const Values &values, const size_t num_steps);

  /** Print tangent-vector values in a compact table. */
  static void PrintDelta(const VectorValues &values, const size_t num_steps);

  /** Export trajectory values to a CSV file. */
  static void ExportValues(const Values &values, const size_t num_steps,
                           const std::string &file_path);

  /** Export tangent-vector values to a plain text file. */
  static void ExportVector(const VectorValues &values, const size_t num_steps,
                           const std::string &file_path);
};

/** Retraction rule specialized for the cart-pole friction cone. */
class CartPoleWithFrictionRetractor : public IERetractor {
 protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithFriction &cp_;

 public:
  /** Constructor. */
  CartPoleWithFrictionRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update and enforce active friction constraints. */
  IEConstraintManifold retract(
      const IEConstraintManifold *manifold, const VectorValues &delta,
      const std::optional<IndexSet> &blocking_indices = {},
      IERetractionInfo *retract_info = nullptr) const override;

  /** Fallback penalty-based retraction for boundary corner cases. */
  IEConstraintManifold retract1(const IEConstraintManifold *manifold,
                                const VectorValues &delta) const;
};

/** Barrier-based retractor for the cart-pole friction cone. */
class CPBarrierRetractor : public IERetractor {
 protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithFriction &cp_;

 public:
  /** Constructor. */
  CPBarrierRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update using a barrier-style local solve. */
  IEConstraintManifold retract(
      const IEConstraintManifold *manifold, const VectorValues &delta,
      const std::optional<IndexSet> &blocking_indices = {},
      IERetractionInfo *retract_info = nullptr) const override;
};

}  // namespace gtdynamics
