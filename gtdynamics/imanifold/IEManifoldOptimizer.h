/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEGradientDescentOptimizer.h
 * @brief First order optimization on manifolds with boundaries/corners.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {
typedef std::map<Key, IEConstraintManifold> IEManifoldValues;
typedef std::map<Key, ConstraintManifold> EManifoldValues;

class IEOptimizer {
public:
  IEOptimizer() {}

  void print() const { std::cout << "Hello\n"; }

  virtual Values optimize(
      const NonlinearFactorGraph &graph,
      const gtdynamics::EqualityConstraints &e_constraints,
      const gtdynamics::InequalityConstraints &i_constraints,
      const gtsam::Values &initial_values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params,
      gtdynamics::ConstrainedOptResult *intermediate_result = nullptr) const {
    auto manifolds = IdentifyManifolds(e_constraints, i_constraints,
                                       initial_values, iecm_params);
    return optimizeManifolds(graph, manifolds, intermediate_result);
  }

  virtual Values optimizeManifolds(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &manifolds,
                                   gtdynamics::ConstrainedOptResult *
                                       intermediate_result = nullptr) const = 0;

public:
  static std::map<Key, Key>
  Var2ManifoldKeyMap(const IEManifoldValues &manifolds);

  static IEManifoldValues IdentifyManifolds(
      const gtdynamics::EqualityConstraints &e_constraints,
      const gtdynamics::InequalityConstraints &i_constraints,
      const gtsam::Values &values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params);

  static Values CollectManifoldValues(const IEManifoldValues &manifolds);

  static VectorValues ComputeTangentVector(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  /// Note: xi is given with the e_basis that only consider equality
  /// constraints.
  static std::pair<IndexSetMap, VectorValues>
  ProjectTangentCone(const IEManifoldValues &manifolds, const VectorValues &v);

  static IEManifoldValues RetractManifolds(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  static Values EManifolds(const IEManifoldValues &manifolds);

  /// Return e_manifolds and constant_e_manifolds.
  static std::pair<Values, Values>
  EManifolds(const IEManifoldValues &manifolds,
             const IndexSetMap &active_indices);

  static bool IsSameMode(const IEManifoldValues &manifolds1,
                         const IEManifoldValues &manifolds2);

  static IndexSetMap
  IdentifyChangeIndices(const IEManifoldValues &manifolds,
                        const IEManifoldValues &new_manifolds);

  static IndexSetMap
  IdentifyApproachingIndices(const IEManifoldValues &manifolds,
                             const IEManifoldValues &new_manifolds,
                             const IndexSetMap &change_indices_map,
                             const double& approach_rate_threshold);

  static IEManifoldValues
  MoveToBoundaries(const IEManifoldValues &manifolds,
                   const IndexSetMap &approach_indices_map);

  static std::string IndicesStr(const IndexSetMap &indices_map);

  static std::string IndicesStr(const IEManifoldValues &manifolds);

  typedef std::function<void(const Values &values, const size_t num_steps)>
      PrintValuesFunc;
  typedef std::function<void(const VectorValues &values,
                             const size_t num_steps)>
      PrintDeltaFunc;

  template <typename IterDetails>
  static void PrintIterDetails(const IterDetails &iter_details,
                               const size_t num_steps,
                               bool print_values = false,
                               PrintValuesFunc print_values_func = NULL,
                               PrintDeltaFunc print_delta_func = NULL) {
    std::string red = "1;31";
    std::string green = "1;32";
    std::string blue = "1;34";

    const auto &state = iter_details.state;
    std::cout << "<======================= Iter "
              << iter_details.state.iterations << " =======================>\n";

    /// Print state
    std::cout << "\033[" + green + "merror: " << std::setprecision(4)
              << state.error << "\033[0m\n";
    auto state_current_str = IEOptimizer::IndicesStr(state.manifolds);
    if (state_current_str.size() > 0) {
      std::cout << "current: " << state_current_str << "\n";
    }
    auto state_grad_blocking_str =
        IEOptimizer::IndicesStr(state.blocking_indices_map);
    if (state_grad_blocking_str.size() > 0) {
      std::cout << "grad blocking: " << state_grad_blocking_str << "\n";
    }

    if (print_values) {
      std::cout << "values: \n";
      print_values_func(IEOptimizer::CollectManifoldValues(state.manifolds),
                        num_steps);

      std::cout << "gradient: \n";
      print_delta_func(
          IEOptimizer::ComputeTangentVector(state.manifolds, state.gradient),
          num_steps);
    }

    /// Print trials
    for (const auto &trial : iter_details.trials) {
      std::string color = trial.step_is_successful ? red : blue;
      std::cout << "\033[" + color + "mlambda: " << trial.lambda
                << "\terror: " << state.error << " -> " << trial.new_error
                << "\tfidelity: " << trial.model_fidelity
                << "\tlinear: " << trial.linear_cost_change
                << "\tnonlinear: " << trial.nonlinear_cost_change
                << "\033[0m\n";

      auto blocking_str = IEOptimizer::IndicesStr(trial.blocking_indices_map);
      if (blocking_str.size() > 0) {
        std::cout << "blocking: " << blocking_str << "\n";
      }
      auto forced_str = IEOptimizer::IndicesStr(trial.forced_indices_map);
      if (forced_str.size() > 0) {
        std::cout << "forced: " << forced_str << "\n";
      }
      auto new_str = IEOptimizer::IndicesStr(trial.new_manifolds);
      if (new_str.size() > 0) {
        std::cout << "new: " << new_str << "\n";
      }

      if (print_values) {
        if (trial.tangent_vector.size() > 0) {
          std::cout << "tangent vector: \n";
          print_delta_func(trial.tangent_vector, num_steps);
        }

        std::cout << "new values: \n";
        print_values_func(
            IEOptimizer::CollectManifoldValues(trial.new_manifolds), num_steps);
      }
    }
  }
};

} // namespace gtsam
