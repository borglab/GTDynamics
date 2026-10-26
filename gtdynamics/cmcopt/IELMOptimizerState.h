/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizerState.h
 * @brief   State and Trial for IELMOptimizer
 * @author  Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>

namespace gtdynamics {
using namespace gtsam;


struct IELMState;
struct IELMTrial;
struct IELMIterationDetails;
struct IELMParams;

/** State corresponding to each LM iteration. */
struct IELMState {
public:
  IEManifoldValues manifolds;
  Values values;
  KeySet unconstrainedKeys;
  double error = 0;
  double lambda = 0;
  double lambdaFactor = 0;
  size_t iterations = 0;
  size_t totalInnerIterations = 0;
  VectorValues gradient;
  IndexSetMap gradientBlockingIndicesMap; // blocking indices map by neg grad
  GaussianFactorGraph::shared_ptr baseLinear;
  GaussianFactorGraph::shared_ptr linearManifoldGraph;
  LinearInequalityConstraints linearBaseInequalityConstraints;
  LinearInequalityConstraints linearManifoldInequalityConstraints;
  IndexSetMapTranslator linearInequalityIndexTranslator;

  /// Default constructor.
  IELMState() {}

  /// Constructor.
  IELMState(const IEManifoldValues &_manifolds,
            const Values &_unconstrained_values,
            const NonlinearFactorGraph &graph,
            const NonlinearFactorGraph &manifold_graph, const double &_lambda,
            const double &_lambdaFactor, size_t _iterations = 0);

  /// Initialize the new state from the result of previous iteration.
  static IELMState fromLastIteration(const IELMIterationDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const NonlinearFactorGraph &manifold_graph,
                                     const LevenbergMarquardtParams &params);

  /// Values of unconstrained variables.
  Values unconstrainedValues() const;

  /// Values of all original constrained opt problem variables.
  Values baseValues() const;

  static double evaluateGraphError(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &_manifolds,
                                   const Values &_unconstrained_values);

  /// Compute the hessian diagonal of the linearized graph. It is used to
  /// construct metric sigma for projection.
  VectorValues computeMetricSigmas(const NonlinearFactorGraph &graph) const;

protected:
  static Values allValues(const IEManifoldValues &manifolds,
                          const Values &unconstrainedValues);

  void construct(const NonlinearFactorGraph &graph,
                 const NonlinearFactorGraph &manifold_graph);

  void linearizeIConstraints();

  /// Compute gradient and identify the blocking constraints by neg grad.
  void computeGradient(const NonlinearFactorGraph &manifold_graph);
};

/** Trial for LM inner iteration with certain lambda setting. */
struct IELMTrial {
  struct LinearUpdate;
  struct NonlinearUpdate;

  /// Perform a trial of iteration with specified lambda value.
  IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
            const double &lambda, const IELMParams &params);

  /// Perform a trial by moving to specified boundaries.
  IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
            const IndexSetMap &forcedIndicesMap);

  struct LinearUpdate {
    double lambda;
    IndexSetMap blockingIndicesMap;
    VectorValues delta;
    VectorValues tangentVector;
    double oldError = 0;
    double newError = 0;
    double costChange = 0;
    bool solveSuccessful = 0;
    size_t numSolves = 0; // number of solving linear systems

    /** Default constructor. */
    LinearUpdate() {}

    /** The function computes:
     * 1) linear update as (approx) solution the i-constrained qp problem
     * 2) blocking constraints as of solving the IQP problem
     * 3) e-manifolds corresponding to the e-constraints and blocking
     * constraints 4) corresponding linear cost change 5) solveSuccessful
     * indicating if solving the linear system is successful
     */
    LinearUpdate(const double &_lambda, const NonlinearFactorGraph &graph,
                 const IELMState &state, const IELMParams &params);

    static LinearUpdate zero(const IELMState &state);

    /** Generate an initial estimate for the IQP problem using active
     * constraints that include the blocking constraint for neg-gradient.
     * @return [delta, blocking_indices, numSolves, solve_sucessful]
     */
    static std::tuple<VectorValues, IndexSet, size_t, bool>
    initialEstimate(const GaussianFactorGraph &quadratic_cost,
                 const IELMState &state,
                 const LevenbergMarquardtParams &params);

    /** Check that the generated linear update is indeed a valid solution to the
     * IQP problem (using base variables).
     * (1) All linear i-constraints are satisfied.
     * (2) Blocking linear i-constraints are active at delta.
     * (3) (optional) blocking constraints fully block the neg-gradient, e.g.,
     * gradient can be expressed as linear combination of bllcking constraint
     * jacobians.
     * @param state State of the optimizer.
     * @param tangentVector Solution to the IQP problem.
     * @param blocking_indices Indices of constraints that are active in IQP
     * w.r.t. delta.
     */
    static bool checkSolutionValid(const IELMState &state,
                                   const VectorValues &tangentVector,
                                   const IndexSet &blocking_indices);

  protected:
    /** Following functions are adapted from LMOptimizerState. */
    mutable std::vector<LMCachedModel> noiseModelCache;
    LMCachedModel *getCachedModel(size_t dim) const;

    GaussianFactorGraph
    buildDampedSystem(GaussianFactorGraph damped /* gets copied */,
                      const IELMState &state) const;

    GaussianFactorGraph
    buildDampedSystem(GaussianFactorGraph damped, // gets copied
                      const VectorValues &sqrtHessianDiagonal) const;

    GaussianFactorGraph
    buildDampedSystem(const GaussianFactorGraph &linear,
                      const VectorValues &sqrtHessianDiagonal,
                      const IELMState &state,
                      const LevenbergMarquardtParams &params) const;

    VectorValues computeTangentVector(const IELMState &state,
                                      const VectorValues &delta,
                                      const KeySet &unconstrainedKeys) const;
  };

  struct NonlinearUpdate {
    IEManifoldValues newManifolds;
    Values newUnconstrainedValues;
    double newError;
    double costChange;
    size_t numRetractionIterations = 0; // total number of iterations in LM opt.
    std::vector<double> retractionDeviationRates;
    double linearCostChangeWithRetractionDelta = 0;

    /** Default constructor. */
    NonlinearUpdate() {}

    /** Compute the new manifolds using the linear update delta and blocking
     * indices. */
    NonlinearUpdate(const IELMState &state, const LinearUpdate &linearUpdate,
                    const NonlinearFactorGraph &graph);

    NonlinearUpdate(const IELMState &state,
                    const IndexSetMap &forcedIndicesMap,
                    const NonlinearFactorGraph &graph);

    void computeError(const NonlinearFactorGraph &graph,
                      const double &oldError);

    static std::pair<VectorValues, double>
    evaluateRetractionDeviation(const IEConstraintManifold &manifold,
                                const IEConstraintManifold &new_manifold,
                                const VectorValues &tangentVector);
  };

public:
  // linear update
  LinearUpdate linearUpdate;

  // nonlinear update
  NonlinearUpdate nonlinearUpdate;

  // decision making
  IndexSetMap forcedIndicesMap;
  double modelFidelity;
  bool stepIsSuccessful;
  bool stopSearchingLambda;
  double trialTime;

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_lambda, double &newLambdaFactor,
                     const LevenbergMarquardtParams &params) const;

  /// Set lambda as increased values for next trial/state.
  void setIncreasedNextLambda(double &new_lambda, double &newLambdaFactor,
                              const LevenbergMarquardtParams &params) const;

  /// Set lambda as decreased values for next trial/state.
  void setDecreasedNextLambda(double &new_lambda, double &newLambdaFactor,
                              const LevenbergMarquardtParams &params) const;
};

/** Print title of summary info of IELM trials. */
void printIELMTrialTitle();

/** Print summary info of an IELM trial. */
void printIELMTrial(
    const IELMState &state, const IELMTrial &trial, const IELMParams &params,
    bool forced = false,
    const KeyFormatter &key_formatter = GTDKeyFormatter);

struct IELMIterationDetails {
  IELMState state;
  std::vector<IELMTrial> trials;

  IELMIterationDetails(const IELMState &_state) : state(_state), trials() {}
};

class IELMOptimizationDetails : public std::vector<IELMIterationDetails> {
public:
  using base = std::vector<IELMIterationDetails>;
  using base::base;

  void exportFile(const std::string &state_file_path,
                  const std::string &trial_file_path) const;
};

} // namespace gtdynamics
