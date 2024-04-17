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
#include <gtdynamics/cmcopt/IELMOptimizerState.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>

namespace gtsam {

struct IELMParams {
  IELMParams() {}
  double boundary_approach_rate_threshold = 3;
  LevenbergMarquardtParams lm_params;
  size_t iqp_max_iters = 0;
  bool show_active_constraints = false;
  bool active_constraints_group_as_categories = false;
};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class IELMOptimizer : public IEOptimizer {

protected:
  const IELMParams ielm_params_;
  std::shared_ptr<IELMItersDetails> details_;

public:
  typedef std::shared_ptr<IELMOptimizer> shared_ptr;

  const IELMItersDetails &details() const { return *details_; }

  /** Constructor */
  IELMOptimizer(const IELMParams &ielm_params = IELMParams(),
                const IEConstraintManifold::Params::shared_ptr &iecm_params =
                    std::make_shared<IEConstraintManifold::Params>())
      : IEOptimizer(iecm_params), ielm_params_(ielm_params),
        details_(std::make_shared<IELMItersDetails>()) {}

  /** Virtual destructor */
  ~IELMOptimizer() {}

  /** Read-only access the parameters */
  const IELMParams &ielmParams() const { return ielm_params_; }

  /** Perform optimization on manifolds. */
  Values optimizeManifolds(const NonlinearFactorGraph &graph,
                           const IEManifoldValues &manifolds,
                           const Values &unconstrained_values) const override;

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
