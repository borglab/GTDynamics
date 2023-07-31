/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizer.h
 * @brief   A nonlinear manifold optimizer that uses the Levenberg-Marquardt
 * trust-region scheme
 * @author  Yetong Zhang
 */

#pragma once
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IELMOptimizerState.h>
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>
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
class IELMOptimizer : public IEOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters
  std::shared_ptr<std::vector<IELMIterDetails>> details_;

public:
  typedef std::shared_ptr<IELMOptimizer> shared_ptr;

  const std::vector<IELMIterDetails> &details() const { return *details_; }

  void exportDetails(const std::string &folder_path) const;

  /** Constructor */
  IELMOptimizer(
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : IEOptimizer(), params_(params),
        details_(std::make_shared<std::vector<IELMIterDetails>>()) {}

  /** Virtual destructor */
  ~IELMOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  /** Perform optimization on manifolds. */
  Values optimizeManifolds(const NonlinearFactorGraph &graph,
                           const IEManifoldValues &manifolds,
                           gtdynamics::ConstrainedOptResult
                               *intermediate_result = nullptr) const override;

  /** Convergence check including
   * 1) mode change
   * 2) error tol
   * 3) absolute error tol
   * 4) relative error tol
   */
  bool checkConvergence(const IELMState &prev_state,
                        const IELMState &state) const;

  /** Check if lambda is within limits. */
  bool checkLambdaWithinLimits(const double &lambda) const;

  /** Perform one iterate, may need to make several trials. */
  IELMIterDetails iterate(const NonlinearFactorGraph &graph,
                          const IELMState &state) const;

  /** Inner loop, perform a trial with specified lambda parameters, changes
   * trial. */
  void tryLambda(const NonlinearFactorGraph &graph,
                 const IELMState &currentState, IELMTrial &trial) const;

  /** Check if a mode change is required. The following conditions need to be
   * met to enforce a mode change:
   * 1) the mode has stayed constant for consecutive states.
   * 2) the most recent failed trial has resulted in different modes.
   * 3) certain boundaries (corresponding to the different mode) are approached
   * with large enough rate.
   */
  bool checkModeChange(const NonlinearFactorGraph &graph,
                       IELMIterDetails &current_iter_details) const;
};

} // namespace gtsam
