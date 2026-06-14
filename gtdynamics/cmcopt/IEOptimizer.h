/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEOptimizer.h
 * @brief Manifold Optimizer for problems with equality and inequality
 * constraints.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;


class IEOptimizer {
protected:
  IEConstraintManifold::Params::shared_ptr iecm_params_;

public:
  /// Base class for CMC-Opt solvers that optimize over IE manifolds plus any
  /// unconstrained variables left outside the connected components.
  IEOptimizer(const IEConstraintManifold::Params::shared_ptr &iecm_params)
      : iecm_params_(iecm_params) {}

  void print() const { std::cout << "Hello\n"; }

  virtual Values optimize(
      const NonlinearFactorGraph &graph,
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &initial_values) const {
    // Split the original problem into IE manifold variables and free
    // variables before the concrete LM/GD solver takes over.
    auto manifolds = IdentifyManifolds(e_constraints, i_constraints,
                                       initial_values, iecm_params_);
    Values unconstrained_values = IdentifyUnconstrainedValues(
        e_constraints, i_constraints, initial_values);
    return optimizeManifolds(graph, manifolds, unconstrained_values);
  }

  /// Solve the reduced problem once the IE manifold blocks are identified.
  virtual Values optimizeManifolds(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &manifolds,
                                   const Values &unconstrained_values) const = 0;

public:
  /// Map each original variable key to the connected-component manifold key
  /// that owns it.
  static std::map<Key, Key>
  Var2ManifoldKeyMap(const IEManifoldValues &manifolds);

  /// Build one IE manifold per constraint-connected component, following the
  /// thesis CMC component construction in Eqs. (4.33)-(4.34).
  static IEManifoldValues IdentifyManifolds(
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params);

  /// Extract variables that do not participate in any equality or inequality
  /// component.
  static Values IdentifyUnconstrainedValues(
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values);

  /// Lift manifold coordinates xi back to ambient tangent vectors via the
  /// equality-manifold basis B_x.
  static VectorValues ComputeTangentVector(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  /// Project each component direction into its tangent cone, matching thesis
  /// Eq. (4.31) componentwise as in Eq. (4.42).
  /// Note: xi is given in the equality-only basis before cone projection.
  static std::pair<IndexSetMap, VectorValues>
  ProjectTangentCone(const IEManifoldValues &manifolds, const VectorValues &v);

  /// Retract each manifold block independently back to the feasible set.
  static IEManifoldValues RetractManifolds(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  /// Drop the inequality state and expose only the equality manifolds.
  static Values EManifolds(const IEManifoldValues &manifolds);

  /// Treat selected active inequalities as temporary equalities, corresponding
  /// to the active-corner manifold in thesis Eq. (4.10).
  static std::pair<Values, Values>
  EManifolds(const IEManifoldValues &manifolds,
             const IndexSetMap &active_indices);

  /// Two iterates are in the same mode when their active sets agree on every
  /// IE manifold component; modes are the corners defined in thesis Eq. (4.9).
  static bool IsSameMode(const IEManifoldValues &manifolds1,
                         const IEManifoldValues &manifolds2);

  /// Return newly activated inequality indices, i.e. constraints that entered
  /// the active set from thesis Eq. (4.3) after a trial step.
  static IndexSetMap
  IdentifyChangeIndices(const IEManifoldValues &manifolds,
                        const IEManifoldValues &new_manifolds);

  /// From manifolds to new manifolds, check which boundaries are approached
  /// with rate larger than the threshold. If multiple boundaries are approached
  /// for a manifold, pick the boundary with max approach rate.
  static IndexSetMap
  IdentifyApproachingIndices(const IEManifoldValues &manifolds,
                             const IEManifoldValues &new_manifolds,
                             const IndexSetMap &change_indices_map,
                             const double &approach_rate_threshold);

  static std::string
  IndicesStr(const IndexSetMap &indices_map,
             const KeyFormatter &keyFormatter = DefaultKeyFormatter);

  static std::string
  IndicesStr(const IEManifoldValues &manifolds,
             const KeyFormatter &keyFormatter = DefaultKeyFormatter);

  typedef std::function<void(const Values &values, const size_t num_steps)>
      PrintValuesFunc;
  typedef std::function<void(const VectorValues &values,
                             const size_t num_steps)>
      PrintDeltaFunc;

  template <typename IterDetails>
  static void
  PrintIterDetails(const IterDetails &iter_details, const size_t num_steps,
                   bool print_values = false,
                   PrintValuesFunc print_values_func = NULL,
                   PrintDeltaFunc print_delta_func = NULL,
                   const KeyFormatter &keyFormatter = DefaultKeyFormatter) {
    std::string red = "1;31";
    std::string green = "1;32";
    std::string blue = "1;34";

    const auto &state = iter_details.state;
    std::cout << "<======================= Iter "
              << iter_details.state.iterations << " =======================>\n";

    /// Print state
    std::cout << "\033[" + green + "merror: " << std::setprecision(4)
              << state.error << "\033[0m\n";
    auto state_current_str =
        IEOptimizer::IndicesStr(state.manifolds, keyFormatter);
    if (state_current_str.size() > 0) {
      std::cout << "current: " << state_current_str << "\n";
    }
    auto state_grad_blocking_str =
        IEOptimizer::IndicesStr(state.grad_blocking_indices_map, keyFormatter);
    if (state_grad_blocking_str.size() > 0) {
      std::cout << "grad blocking: " << state_grad_blocking_str << "\n";
    }

    for (const auto &it : state.manifolds) {
      double i_error = it.second.evalIViolation();
      double e_error = it.second.evalEViolation();
      if (e_error > 1e-5) {
        std::cout << "violating e: " << keyFormatter(it.first) << " " << e_error
                  << "\n";
      }
      if (i_error > 1e-5) {
        std::cout << "violating i: " << keyFormatter(it.first) << " " << i_error
                  << "\n";
      }
    }

    if (print_values) {
      std::cout << "values: \n";
      print_values_func(state.manifolds.baseValues(), num_steps);

      std::cout << "gradient: \n";
      print_delta_func(
          IEOptimizer::ComputeTangentVector(state.manifolds, state.gradient),
          num_steps);
    }

    /// Print trials
    for (const auto &trial : iter_details.trials) {
      std::string color = trial.step_is_successful ? red : blue;
      std::cout << "\033[" + color + "mlambda: " << trial.linear_update.lambda
                << "\terror: " << state.error << " -> "
                << trial.nonlinear_update.new_error
                << "\tfidelity: " << trial.model_fidelity
                << "\tlinear: " << trial.linear_update.cost_change
                << "\tnonlinear: " << trial.nonlinear_update.cost_change
                << "\033[0m\n";

      auto blocking_str = IEOptimizer::IndicesStr(
          trial.linear_update.blocking_indices_map, keyFormatter);
      if (blocking_str.size() > 0) {
        std::cout << "blocking: " << blocking_str << "\n";
      }
      auto forced_str =
          IEOptimizer::IndicesStr(trial.forced_indices_map, keyFormatter);
      if (forced_str.size() > 0) {
        std::cout << "forced: " << forced_str << "\n";
      }
      auto new_str = IEOptimizer::IndicesStr(
          trial.nonlinear_update.new_manifolds, keyFormatter);
      if (new_str.size() > 0) {
        std::cout << "new: " << new_str << "\n";
      }

      if (print_values) {
        if (trial.linear_update.tangent_vector.size() > 0) {
          std::cout << "tangent vector: \n";
          print_delta_func(trial.linear_update.tangent_vector, num_steps);
        }

        std::cout << "new values: \n";
        print_values_func(
            trial.nonlinear_update.new_manifolds.baseValues(),
            num_steps);
      }
    }
  }
};

} // namespace gtdynamics
