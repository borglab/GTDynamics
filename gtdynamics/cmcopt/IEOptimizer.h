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

#include <gtdynamics/cmcopt/CMPoint.h>
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
  /// Base class for CMC-Opt solvers that optimize over IE points plus any
  /// unconstrained variables left outside the connected components.
  IEOptimizer(const IEConstraintManifold::Params::shared_ptr &iecm_params)
      : iecm_params_(iecm_params) {}

  void print() const { std::cout << "Hello\n"; }

  virtual Values optimize(
      const NonlinearFactorGraph &graph,
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &initial_values) const {
    // Split the original problem into CMPoint values and free
    // variables before the concrete LM/GD solver takes over.
    auto [types, points] = identifyConnectedComponents(
        e_constraints, i_constraints,
        initial_values, iecm_params_);
    Values unconstrainedValues = identifyUnconstrainedValues(
        e_constraints, i_constraints, initial_values);
    return optimize(graph, points, unconstrainedValues);
  }

  /// Solve the reduced problem once the IE manifold blocks are identified.
  virtual Values optimize(const NonlinearFactorGraph &graph,
                                   const CMPoints &points,
                                   const Values &unconstrainedValues) const = 0;

public:
  /// Transform ambient cost factors to CMPoint variables without legacy CM
  /// values.
  static NonlinearFactorGraph pointGraph(const NonlinearFactorGraph &graph,
                                         const CMPoints &points);
  /// Reject ambiguous ownership and component-key collisions before
  /// optimization.
  static void validatePoints(const CMPoints &points,
                             const Values &unconstrained);

  /// Map each original variable key to the connected-component point key
  /// that owns it.
  static std::map<Key, Key>
  varToManifoldKeyMap(const CMPoints &points);

  /** Discovered runtime definitions and their initial points. Each point
   * references its type; types and points need not have matching indices/counts.
   */
  struct Components {
    std::vector<IEConstraintManifold::shared_ptr> types;
    CMPoints points;
  };

  /// Discover constraint-connected components and construct their types/points.
  /// Equivalent definitions are not yet shared automatically.
  static Components identifyConnectedComponents(
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params);

  /// Extract variables that do not participate in any equality or inequality
  /// component.
  static Values identifyUnconstrainedValues(
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values);

  /// Lift manifold coordinates xi back to ambient tangent vectors via the
  /// equality-manifold basis B_x.
  static VectorValues computeTangentVector(const CMPoints &points,
                                           const VectorValues &delta);

  /// Project each component direction into its tangent cone, matching thesis
  /// Eq. (4.31) componentwise as in Eq. (4.42).
  /// Note: xi is given in the equality-only basis before cone projection.
  static std::pair<IndexSetMap, VectorValues>
  projectTangentCone(const CMPoints &points, const VectorValues &v);

  /// Retract each point independently back to the feasible set.
  static CMPoints retractPoints(const CMPoints &points,
                                           const VectorValues &delta);

  /// Two iterates are in the same mode when their active sets agree on every
  /// IE manifold component; modes are the corners defined in thesis Eq. (4.9).
  static bool isSameMode(const CMPoints &points1,
                         const CMPoints &points2);

  /// Return newly activated inequality indices, i.e. constraints that entered
  /// the active set from thesis Eq. (4.3) after a trial step.
  static IndexSetMap
  identifyChangeIndices(const CMPoints &points,
                        const CMPoints &newPoints);

  /// From points to new points, check which boundaries are approached
  /// with rate larger than the threshold. If multiple boundaries are approached
  /// for a manifold, pick the boundary with max approach rate.
  static IndexSetMap
  identifyApproachingIndices(const CMPoints &points,
                             const CMPoints &newPoints,
                             const IndexSetMap &change_indices_map,
                             const double &approach_rate_threshold);

  static std::string
  indicesString(const IndexSetMap &indices_map,
             const KeyFormatter &keyFormatter = DefaultKeyFormatter);

  static std::string
  indicesString(const CMPoints &points,
             const KeyFormatter &keyFormatter = DefaultKeyFormatter);

  typedef std::function<void(const Values &values, const size_t num_steps)>
      PrintValuesFunction;
  typedef std::function<void(const VectorValues &values,
                             const size_t num_steps)>
      PrintDeltaFunction;

  template <typename IterDetails>
  static void
  printIterationDetails(const IterDetails &iter_details, const size_t num_steps,
                   bool print_values = false,
                   PrintValuesFunction print_values_func = NULL,
                   PrintDeltaFunction print_delta_func = NULL,
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
        IEOptimizer::indicesString(state.points, keyFormatter);
    if (state_current_str.size() > 0) {
      std::cout << "current: " << state_current_str << "\n";
    }
    for (const auto &it : state.points) {
      double i_error = it.second.evaluateInequalityViolation();
      double e_error = it.second.evaluateEqualityViolation();
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
      print_values_func(ambientValues(state.points), num_steps);

      std::cout << "gradient: \n";
      print_delta_func(
          IEOptimizer::computeTangentVector(state.points, state.gradient),
          num_steps);
    }

    /// Print trials
    for (const auto &trial : iter_details.trials) {
      std::string color = trial.stepIsSuccessful ? red : blue;
      std::cout << "\033[" + color + "mlambda: " << trial.linearUpdate.lambda
                << "\terror: " << state.error << " -> "
                << trial.nonlinearUpdate.newError
                << "\tfidelity: " << trial.modelFidelity
                << "\tlinear: " << trial.linearUpdate.costChange
                << "\tnonlinear: " << trial.nonlinearUpdate.costChange
                << "\033[0m\n";

      auto blocking_str = IEOptimizer::indicesString(
          trial.linearUpdate.blockingIndicesMap, keyFormatter);
      if (blocking_str.size() > 0) {
        std::cout << "blocking: " << blocking_str << "\n";
      }
      auto forced_str =
          IEOptimizer::indicesString(trial.forcedIndicesMap, keyFormatter);
      if (forced_str.size() > 0) {
        std::cout << "forced: " << forced_str << "\n";
      }
      auto new_str = IEOptimizer::indicesString(
          trial.nonlinearUpdate.newPoints, keyFormatter);
      if (new_str.size() > 0) {
        std::cout << "new: " << new_str << "\n";
      }

      if (print_values) {
        if (trial.linearUpdate.tangentVector.size() > 0) {
          std::cout << "tangent vector: \n";
          print_delta_func(trial.linearUpdate.tangentVector, num_steps);
        }

        std::cout << "new values: \n";
        print_values_func(ambientValues(trial.nonlinearUpdate.newPoints),
            num_steps);
      }
    }
  }
};

} // namespace gtdynamics
