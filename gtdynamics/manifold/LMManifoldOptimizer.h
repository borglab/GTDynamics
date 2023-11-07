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
#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/LMManifoldOptimizerState.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class LMManifoldOptimizer : public ManifoldOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters
  std::shared_ptr<LMItersDetails> details_;

public:
  typedef std::shared_ptr<LMManifoldOptimizer> shared_ptr;

  const LMItersDetails &details() const { return *details_; }

  /** Constructor */
  LMManifoldOptimizer(
      const ManifoldOptimizerParameters &mopt_params,
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : ManifoldOptimizer(mopt_params), params_(params),
        details_(std::make_shared<LMItersDetails>()) {}

  /** Virtual destructor */
  ~LMManifoldOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  /** Run manifold optimization by substituting the constrained variables with
   * the constraint manifold variables. */
  virtual gtsam::Values
  optimize(const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &constraints,
           const gtsam::Values &initial_values,
           gtdynamics::ConstrainedOptResult *intermediate_result =
               nullptr) const override;

  /// Optimization given manifold optimization problem.
  gtsam::Values optimize(
      const NonlinearFactorGraph &graph, const ManifoldOptProblem &mopt_problem,
      gtdynamics::ConstrainedOptResult *intermediate_result = nullptr) const;

  /** Perform one iterate, may need to make several trials. */
  LMIterDetails iterate(const NonlinearFactorGraph &graph,
                        const NonlinearFactorGraph &manifold_graph,
                        const LMState &state) const;

  /** Convergence check including
   * 1) mode change
   * 2) error tol
   * 3) absolute error tol
   * 4) relative error tol
   */
  bool checkConvergence(const LMState &prev_state, const LMState &state) const;

  /** Check if lambda is within limits. */
  bool checkLambdaWithinLimits(const double &lambda) const;
};

} // namespace gtsam
