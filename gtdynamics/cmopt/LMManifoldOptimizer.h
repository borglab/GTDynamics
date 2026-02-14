/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LMManifoldOptimizer.h
 * @brief   A nonlinear manifold optimizer that uses the Levenberg-Marquardt
 * trust-region scheme
 * @author  Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/cmopt/LMManifoldOptimizerState.h>
#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

/**
 * Levenberg-Marquardt optimizer over transformed manifold variables.
 *
 * This optimizer runs LM on the manifold optimization problem produced by
 * `ManifoldOptimizer` and keeps detailed trial/state bookkeeping in
 * `LMManifoldOptimizerState`.
 *
 * It is the explicit LM implementation path in `cmopt`, complementary to the
 * generic `NonlinearMOptimizer` wrapper around standard GTSAM optimizers.
 *
 * @see README.md#solvers
 * @see ManifoldOptimizer
 * @see LMState
 * @see LMTrial
 */
class LMManifoldOptimizer : public ManifoldOptimizer {
 protected:
  const LevenbergMarquardtParams params_;  ///< LM parameters
  std::shared_ptr<LMItersDetails> details_;

 public:
  typedef std::shared_ptr<LMManifoldOptimizer> shared_ptr;

  const LMItersDetails &details() const { return *details_; }

  /**
   * Constructor.
   * @param mopt_params Parameters for manifold construction/transformation.
   * @param params Levenberg-Marquardt parameters for manifold optimization.
   */
  LMManifoldOptimizer(
      const ManifoldOptimizerParameters &mopt_params,
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : ManifoldOptimizer(mopt_params),
        params_(params),
        details_(std::make_shared<LMItersDetails>()) {}

  /** Virtual destructor */
  ~LMManifoldOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  /**
   * Run manifold optimization from a constrained problem definition.
   * @param graph Cost factor graph.
   * @param constraints Equality constraints.
   * @param initial_values Initial values for all variables.
   * @return Optimized base-variable values.
   */
  virtual gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph &graph,
      const NonlinearEqualityConstraints &constraints,
      const gtsam::Values &initial_values) const override;

  /**
   * Optimize an already transformed manifold optimization problem.
   * @param graph Original base cost graph, used for error evaluation.
   * @param mopt_problem Transformed manifold optimization problem.
   * @return Optimized base-variable values.
   */
  gtsam::Values optimize(const NonlinearFactorGraph &graph,
                         const ManifoldOptProblem &mopt_problem) const;

  /**
   * Perform one outer LM iteration, potentially with multiple lambda trials.
   * @param graph Original base cost graph.
   * @param manifold_graph Transformed manifold graph.
   * @param state Current optimizer state.
   * @return Iteration details including all trials.
   */
  LMIterDetails iterate(const NonlinearFactorGraph &graph,
                        const NonlinearFactorGraph &manifold_graph,
                        const LMState &state) const;

  /**
   * Check convergence between two consecutive states.
   * @param prev_state Previous state.
   * @param state Current state.
   * @return True if convergence criteria are met.
   */
  bool checkConvergence(const LMState &prev_state, const LMState &state) const;

  /// Check whether lambda is within configured bounds.
  bool checkLambdaWithinLimits(const double &lambda) const;
};

}  // namespace gtdynamics
