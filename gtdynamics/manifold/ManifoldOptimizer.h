/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizer.h
 * @brief Manifold Optimzier with manually written LM algorithm.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

using gtdynamics::EqConsOptProblem, gtdynamics::EqualityConstraints;

namespace gtsam {


/// Manifold optimization problem.
struct ManifoldOptProblem {
  NonlinearFactorGraph graph_;  // cost function on constraint manifolds
  std::vector<EqualityConstraints::shared_ptr>
      components_;  // All the constraint-connected components
  Values
      values_;  // values for constraint manifolds and unconstrained variables
  Values fixed_manifolds_;     // fully constrained variables
  KeySet unconstrained_keys_;  // keys for unconstrained variables
  KeySet manifold_keys_;       // keys for constraint manifold variables

  /** Dimension of the manifold optimization problem, as factor dimension x
   * variable dimension. */
  std::pair<size_t, size_t> problemDimension() const;

  Values unconstrainedValues() const;

  EManifoldValues manifolds() const;

  EManifoldValues constManifolds() const;

  /// Customizable print function.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
};



/// Parameters for manifold optimizer.
struct ManifoldOptimizerParameters
    : public gtdynamics::ConstrainedOptimizationParameters {
  using Base = gtdynamics::ConstrainedOptimizationParameters;
  ConstraintManifold::Params::shared_ptr
      cc_params;               // Parameter for constraint-connected components
  bool retract_init = true;    // Perform retraction on constructing values for
                               // connected component.
  bool retract_final = false;  // Perform retraction on manifolds after
                               // optimization, used for infeasible methods.
  /// Default Constructor.
  ManifoldOptimizerParameters();
};

/// Base class for manifold optimizer.
class ManifoldOptimizer : public gtdynamics::ConstrainedOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const ManifoldOptimizer>;

 protected:
  const ManifoldOptimizerParameters p_;

 public:
  /// Default constructor.
  ManifoldOptimizer() : p_(ManifoldOptimizerParameters()) {}

  /// Construct from parameters.
  ManifoldOptimizer(const ManifoldOptimizerParameters& parameters)
      : p_(parameters) {}

 public:
  /** Perform dfs to find the connected component that contains start_key. Will
   * also erase all the keys in the connected component from keys.
   */
  static EqualityConstraints::shared_ptr
  IdentifyConnectedComponent(const EqualityConstraints &constraints,
                             const Key start_key, KeySet &keys,
                             const VariableIndex &var_index);

  /// Identify the connected components by constraints.
  static std::vector<EqualityConstraints::shared_ptr>
  IdentifyConnectedComponents(const EqualityConstraints &constraints);

  /// Create equivalent factor graph on manifold variables.
  static NonlinearFactorGraph ManifoldGraph(const NonlinearFactorGraph &graph,
                                            const std::map<Key, Key> &var2man_keymap,
                                            const Values& fc_manifolds = Values());

  /** Create values for the manifold optimization probelm by (1) create
   * constraint manifolds for constraint-connected components; (2) identify if
   * the cosntraint manifold is fully constrained; (3) collect unconstrained
   * variables.
   */
  void constructMoptValues(const EqConsOptProblem &ecopt_problem,
                           ManifoldOptProblem &mopt_problem) const;

  /// Create initial values for the constraint manifold variables.
  void constructManifoldValues(const EqConsOptProblem &ecopt_problem,
                               ManifoldOptProblem &mopt_problem) const;

  /// Collect values for unconstrained variables.
  static void
  constructUnconstrainedValues(const EqConsOptProblem &ecopt_problem,
                               ManifoldOptProblem &mopt_problem);

  /** Create a factor graph of cost function with the constraint manifold
   * variables. */
  static void constructMoptGraph(const EqConsOptProblem &ecopt_problem,
                                 ManifoldOptProblem &mopt_problem);

  /** Transform an equality-constrained optimization problem into a manifold
   * optimization problem by creating constraint manifolds. */
  ManifoldOptProblem
  problemTransform(const EqConsOptProblem &ecopt_problem) const;

  /// Initialize the manifold optization problem.
  ManifoldOptProblem
  initializeMoptProblem(const NonlinearFactorGraph &costs,
                        const EqualityConstraints &constraints,
                        const Values &init_values) const;

  /// Construct values of original variables.
  Values baseValues(const ManifoldOptProblem &mopt_problem,
                    const Values &nopt_values) const;

  VectorValues baseTangentVector(const ManifoldOptProblem &mopt_problem,
                                 const Values &values,
                                 const VectorValues &delta) const;

};

}  // namespace gtsam
