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

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtdynamics {

using gtsam::DefaultKeyFormatter;
using gtsam::Key;
using gtsam::KeyFormatter;
using gtsam::KeySet;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using gtsam::VariableIndex;
using gtsam::VectorValues;

using gtsam::NonlinearEqualityConstraints;

/**
 * Transformed manifold optimization problem data.
 *
 * This holds the transformed factor graph, manifold/unconstrained values, and
 * bookkeeping sets needed to move between transformed and base variable spaces.
 *
 * @see README.md#ccc-transformation
 * @see README.md#equivalent-factors
 */
struct ManifoldOptProblem {
  NonlinearFactorGraph graph_;  // cost function on constraint manifolds
  std::vector<NonlinearEqualityConstraints::shared_ptr>
      components_;  // All the constraint-connected components
  Values
      values_;  // values for constraint manifolds and unconstrained variables
  Values fixed_manifolds_;     // fully constrained variables
  KeySet unconstrained_keys_;  // keys for unconstrained variables
  KeySet manifold_keys_;       // keys for constraint manifold variables

  /** Dimension of the manifold optimization problem, as factor dimension x
   * variable dimension.
   * @return Pair of `(total_factor_dimension, total_variable_dimension)`.
   */
  std::pair<size_t, size_t> problemDimension() const;

  /// Return values of unconstrained variables only.
  Values unconstrainedValues() const;

  /// Return free (non-fixed) constraint manifold values.
  EManifoldValues manifolds() const;

  /// Return fixed (zero-dimensional) manifold values.
  EManifoldValues constManifolds() const;

  /**
   * Print a summary of manifold optimization problem contents.
   * @param s Prefix string.
   * @param keyFormatter Key formatter.
   */
  void print(const std::string &s = "",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const;
};

/**
 * Parameters controlling manifold problem construction and post-processing.
 *
 * @see README.md#ccc-transformation
 * @see README.md#retraction
 */
struct ManifoldOptimizerParameters : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  ConstraintManifold::Params::shared_ptr
      cc_params;               // Parameter for constraint-connected components
  bool retract_init = true;    // Perform retraction on constructing values for
                               // connected component.
  bool retract_final = false;  // Perform retraction on manifolds after
                               // optimization, used for infeasible methods.
  /// Default Constructor.
  ManifoldOptimizerParameters();
};

/**
 * Base class that transforms equality-constrained problems into manifold form.
 *
 * This class identifies constraint-connected components, builds
 * `ConstraintManifold` values, and replaces base factors with equivalent
 * factors over manifold variables.
 *
 * @see README.md#ccc-transformation
 * @see README.md#equivalent-factors
 */
class ManifoldOptimizer : public ConstrainedOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const ManifoldOptimizer>;

 protected:
  const ManifoldOptimizerParameters p_;

 public:
  /// Default constructor.
  ManifoldOptimizer() : p_(ManifoldOptimizerParameters()) {}

  /// Construct from parameters.
  ManifoldOptimizer(const ManifoldOptimizerParameters &parameters)
      : p_(parameters) {}

 public:
  /**
   * Find a connected component containing `start_key` using DFS.
   * @param constraints Equality constraints.
   * @param start_key Starting key for DFS.
   * @param keys In/out set of unexplored keys; keys in the found component are
   * removed.
   * @param var_index Variable index over constraints.
   * @return Constraint subset corresponding to the connected component.
   */
  static NonlinearEqualityConstraints::shared_ptr IdentifyConnectedComponent(
      const NonlinearEqualityConstraints &constraints, const Key start_key, KeySet &keys,
      const VariableIndex &var_index);

  /// Identify the connected components by constraints.
  static std::vector<NonlinearEqualityConstraints::shared_ptr>
  IdentifyConnectedComponents(const NonlinearEqualityConstraints &constraints);

  /**
   * Create equivalent cost graph on manifold variables.
   * @param graph Original factor graph.
   * @param var2man_keymap Mapping from base variable key to manifold key.
   * @param fc_manifolds Values of fully constrained manifolds.
   * @return Transformed manifold graph.
   */
  static NonlinearFactorGraph ManifoldGraph(
      const NonlinearFactorGraph &graph,
      const std::map<Key, Key> &var2man_keymap,
      const Values &fc_manifolds = Values());

  /** Create values for the manifold optimization problem by (1) create
   * constraint manifolds for constraint-connected components; (2) identify if
   * the constraint manifold is fully constrained; (3) collect unconstrained
   * variables.
   * @param equalityConstrainedProblem Input constrained problem.
   * @param mopt_problem Output manifold optimization problem.
   */
  void constructMoptValues(const EConsOptProblem &equalityConstrainedProblem,
                           ManifoldOptProblem &mopt_problem) const;

  /**
   * Create initial values for constraint manifold variables.
   * @param equalityConstrainedProblem Input constrained problem.
   * @param mopt_problem Output manifold optimization problem.
   */
  void constructManifoldValues(
      const EConsOptProblem &equalityConstrainedProblem,
      ManifoldOptProblem &mopt_problem) const;

  /**
   * Collect initial values for unconstrained variables.
   * @param equalityConstrainedProblem Input constrained problem.
   * @param mopt_problem Output manifold optimization problem.
   */
  static void constructUnconstrainedValues(
      const EConsOptProblem &equalityConstrainedProblem,
      ManifoldOptProblem &mopt_problem);

  /** Create a factor graph of cost function with the constraint manifold
   * variables.
   * @param equalityConstrainedProblem Input constrained problem.
   * @param mopt_problem Output manifold optimization problem.
   */
  static void constructMoptGraph(
      const EConsOptProblem &equalityConstrainedProblem,
      ManifoldOptProblem &mopt_problem);

  /** Transform an equality-constrained optimization problem into a manifold
   * optimization problem by creating constraint manifolds.
   * @param equalityConstrainedProblem Input constrained problem.
   * @return Transformed manifold optimization problem.
   */
  ManifoldOptProblem problemTransform(
      const EConsOptProblem &equalityConstrainedProblem) const;

  /**
   * Initialize and transform an equality-constrained problem.
   * @param costs Cost factor graph.
   * @param constraints Equality constraints.
   * @param init_values Initial values.
   * @return Initialized manifold optimization problem.
   */
  ManifoldOptProblem initializeMoptProblem(
      const NonlinearFactorGraph &costs, const NonlinearEqualityConstraints &constraints,
      const Values &init_values) const;

  /**
   * Reconstruct base variable values from optimized manifold values.
   * @param mopt_problem Transformed manifold optimization problem.
   * @param nopt_values Optimized values in transformed variable space.
   * @return Values in original base variable space.
   */
  Values baseValues(const ManifoldOptProblem &mopt_problem,
                    const Values &nopt_values) const;

  /**
   * Lift transformed-space delta to base-space tangent vectors.
   * @param mopt_problem Transformed manifold optimization problem.
   * @param values Current transformed-space values.
   * @param delta Delta in transformed variable space.
   * @return Tangent vectors in base variable space.
   */
  VectorValues baseTangentVector(const ManifoldOptProblem &mopt_problem,
                                 const Values &values,
                                 const VectorValues &delta) const;
};

}  // namespace gtdynamics
