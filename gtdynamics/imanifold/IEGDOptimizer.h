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
#include <gtdynamics/imanifold/IEOptimizer.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {

struct IEGDState;
struct IEGDTrial;
struct IEGDIterDetails;

struct GDParams {
  double alpha = 0.2;
  double beta = 0.5;
  double init_lambda = 1;
  double absoluteErrorTol = 1e-9;
  double relativeErrorTol = 1e-9;
  double errorTol = 1e-9;
  size_t maxIterations = 100;
  double muLowerBound = 1e-5;
  double boundary_approach_rate_threshold = 5;
  bool verbose = false;
};

struct IEGDState {
public:
  IEManifoldValues manifolds;
  double error = 0;
  Values e_manifolds;
  VectorValues gradient;
  VectorValues descent_dir;
  VectorValues projected_descent_dir;
  IndexSetMap blocking_indices_map;
  double lambda = 0; // step length
  size_t iterations = 0;
  size_t totalNumberInnerIterations = 0;

  IEGDState() {}

  /// Constructor.
  IEGDState(const IEManifoldValues &_manifolds,
            const NonlinearFactorGraph &graph, size_t _iterations = 0)
      : manifolds(_manifolds),
        error(graph.error(manifolds.baseValues())),
        e_manifolds(IEOptimizer::EManifolds(_manifolds)),
        iterations(_iterations) {
    computeDescentDirection(graph);
  }

  /// Initialize the new state from the result of previous iteration.
  static IEGDState FromLastIteration(const IEGDIterDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const GDParams &params);

  /// compute gradient, then project neg grad into tangent cone.
  void computeDescentDirection(const NonlinearFactorGraph &graph);
};

/** Trial for GD inner iteration with certain lambda setting. */
struct IEGDTrial {
  IEManifoldValues new_manifolds;
  IndexSetMap forced_indices_map;
  VectorValues delta;
  VectorValues tangent_vector;
  IndexSetMap blocking_indices_map;

  double linear_cost_change;
  double new_error;
  double nonlinear_cost_change;
  double model_fidelity;

  bool step_is_successful;
  double lambda;

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_mu, const GDParams &params) const;

  /// Compute the linear update delta and tangent vector.
  void computeDelta(const IEGDState &state);

  /** Compute the new manifolds using the linear update delta and blocking
   * indices. */
  void computeNewManifolds(const IEGDState &state);

  void computeNewError(const NonlinearFactorGraph &graph,
                       const IEGDState &state);

  /** Print summary info of the trial. */
  void print(const IEGDState &state) const;
};

struct IEGDIterDetails {
  IEGDState state;
  std::vector<IEGDTrial> trials;

  IEGDIterDetails(const IEGDState &_state) : state(_state), trials() {}
};

typedef std::vector<IEGDIterDetails> IEGDItersDetails;

class IEGDOptimizer : public IEOptimizer {

protected:
  const GDParams params_; ///< LM parameters
  std::shared_ptr<IEGDItersDetails> details_;

public:
  /** Constructor */
  IEGDOptimizer(const GDParams &params = GDParams(),
                const IEConstraintManifold::Params::shared_ptr &iecm_params =
                    std::make_shared<IEConstraintManifold::Params>())
      : IEOptimizer(iecm_params), params_(params),
        details_(std::make_shared<IEGDItersDetails>()) {}

  const IEGDItersDetails &details() const { return *details_; }

  // IEManifoldValues lineSearch(const NonlinearFactorGraph &graph,
  //                             const IEManifoldValues &manifolds,
  //                             const gtsam::VectorValues &proj_dir,
  //                             const gtsam::VectorValues &descent_dir,
  //                             gtsam::VectorValues &delta) const;

  virtual Values
  optimizeManifolds(const NonlinearFactorGraph &graph,
                    const IEManifoldValues &manifolds,
                    const Values &unconstrained_values,
                    gtdynamics::ConstrainedOptResult *intermediate_result =
                        nullptr) const override;

  /** Perform one iterate, may need to make several trials. */
  IEGDIterDetails iterate(const NonlinearFactorGraph &graph,
                          const IEGDState &state) const;

  /** Inner loop, perform a trial with specified lambda parameters, changes
   * trial. */
  void tryLambda(const NonlinearFactorGraph &graph,
                 const IEGDState &currentState, IEGDTrial &trial) const;

  /** Check if lambda is within limits. */
  bool checkMuWithinLimits(const double &lambda) const;

  /** Check if a mode change is required. The following conditions need to be
   * met to enforce a mode change:
   * 1) the mode has stayed constant for consecutive states.
   * 2) the most recent failed trial has resulted in different modes.
   * 3) certain boundaries (corresponding to the different mode) are approached
   * with large enough rate.
   */
  bool checkModeChange(const NonlinearFactorGraph &graph,
                       IEGDIterDetails &current_iter_details) const;

  /** Convergence check including
   * 1) mode change
   * 2) error tol
   * 3) absolute error tol
   * 4) relative error tol
   */
  bool checkConvergence(const IEGDState &prev_state,
                        const IEGDState &state) const;
};

} // namespace gtsam
