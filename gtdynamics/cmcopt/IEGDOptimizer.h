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

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtdynamics {
using namespace gtsam;


struct IEGDState;
struct IEGDTrial;
struct IEGDIterationDetails;

struct GradientDescentParams {
  double alpha = 0.2;
  double beta = 0.5;
  double initialLambda = 1;
  double absoluteErrorTol = 1e-9;
  double relativeErrorTol = 1e-9;
  double errorTol = 1e-9;
  size_t maxIterations = 100;
  double muLowerBound = 1e-5;
  double boundaryApproachRateThreshold = 5;
  bool verbose = false;
};

struct IEGDState {
public:
  IEManifoldValues manifolds;
  double error = 0;
  Values equalityManifolds;
  VectorValues gradient;
  VectorValues descentDirection;
  VectorValues projectedDescentDirection;
  IndexSetMap blockingIndicesMap;
  double lambda = 0; // step length
  size_t iterations = 0;
  size_t totalInnerIterations = 0;

  IEGDState() {}

  /// Constructor.
  IEGDState(const IEManifoldValues &_manifolds,
            const NonlinearFactorGraph &graph, size_t _iterations = 0)
      : manifolds(_manifolds),
        error(graph.error(manifolds.baseValues())),
        equalityManifolds(IEOptimizer::equalityManifolds(_manifolds)),
        iterations(_iterations) {
    computeDescentDirection(graph);
  }

  /// Initialize the new state from the result of previous iteration.
  static IEGDState fromLastIteration(const IEGDIterationDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const GradientDescentParams &params);

  /// compute gradient, then project neg grad into tangent cone.
  void computeDescentDirection(const NonlinearFactorGraph &graph);

  Values baseValues() const;
};

/** Trial for GD inner iteration with certain lambda setting. */
struct IEGDTrial {
  IEManifoldValues newManifolds;
  IndexSetMap forcedIndicesMap;
  VectorValues delta;
  VectorValues tangentVector;
  IndexSetMap blockingIndicesMap;

  double linearCostChange;
  double newError;
  double nonlinearCostChange;
  double modelFidelity;

  bool stepIsSuccessful;
  double lambda;

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_mu, const GradientDescentParams &params) const;

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

struct IEGDIterationDetails {
  IEGDState state;
  std::vector<IEGDTrial> trials;

  IEGDIterationDetails(const IEGDState &_state) : state(_state), trials() {}
};

typedef std::vector<IEGDIterationDetails> IEGDOptimizationDetails;

class IEGDOptimizer : public IEOptimizer {

protected:
  const GradientDescentParams params_; ///< LM parameters
  std::shared_ptr<IEGDOptimizationDetails> details_;

public:
  /** Constructor */
  IEGDOptimizer(const GradientDescentParams &params = GradientDescentParams(),
                const IEConstraintManifold::Params::shared_ptr &iecm_params =
                    std::make_shared<IEConstraintManifold::Params>())
      : IEOptimizer(iecm_params), params_(params),
        details_(std::make_shared<IEGDOptimizationDetails>()) {}

  const IEGDOptimizationDetails &details() const { return *details_; }

  // IEManifoldValues lineSearch(const NonlinearFactorGraph &graph,
  //                             const IEManifoldValues &manifolds,
  //                             const VectorValues &proj_dir,
  //                             const VectorValues &descentDirection,
  //                             VectorValues &delta) const;

  virtual Values
  optimizeManifolds(const NonlinearFactorGraph &graph,
                    const IEManifoldValues &manifolds,
                    const Values &unconstrainedValues) const override;

  /** Perform one iterate, may need to make several trials. */
  IEGDIterationDetails iterate(const NonlinearFactorGraph &graph,
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
                       IEGDIterationDetails &current_iter_details) const;

  /** Convergence check including
   * 1) mode change
   * 2) error tol
   * 3) absolute error tol
   * 4) relative error tol
   */
  bool checkConvergence(const IEGDState &prev_state,
                        const IEGDState &state) const;
};

} // namespace gtdynamics
