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
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/constrained/QpProblem.h>

#include <limits>

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
  GaussianFactorGraph::shared_ptr baseLinear;
  GaussianFactorGraph::shared_ptr linearManifoldGraph;
  /// Undamped objective and homogeneous active inequalities in B coordinates.
  QpProblem tangentQp;
  /// Contiguous QP rows belonging to one original nonlinear constraint block.
  struct QpConstraintBlock {
    Key component;
    size_t constraint;
    size_t firstRow;
    size_t numRows;
  };
  std::vector<QpConstraintBlock> qpConstraintBlocks;
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
    double lambda = 0;
    IndexSetMap blockingIndicesMap;
    VectorValues delta;
    VectorValues tangentVector;
    double oldError = 0;
    double newError = 0;
    double costChange = 0;
    bool solveSuccessful = 0;
    std::string failureReason;
    /// Main GTSAM working-face solves; excludes initialization projections.
    size_t qpIterations = 0;

    /** Default constructor. */
    LinearUpdate() {}

    /** Compute a damped LM step and its blocking inequality constraints.
     * Solve the global tangent-cone QP with GTSAM's sparse active-set solver.
     * Optionally request feasible descent instead of QP convergence.
     * Equality constraints have already been eliminated by B.
     * Lift the result to ambient tangent coordinates and record the predicted
     * reduction in the undamped cost model.
     */
    LinearUpdate(const double &_lambda, const NonlinearFactorGraph &graph,
                 const IELMState &state, const IELMParams &params);

    /// Construct a zero step for a boundary-mode change.
    static LinearUpdate zero(const IELMState &state);

    /** Check directional feasibility and selected-face tightness in component
     * coordinates. This check does not certify QP stationarity or optimality.
     */
    static bool checkSolutionValid(const IELMState &state,
                                   const VectorValues &delta,
                                   const IndexSetMap &blockingIndices);

   protected:
    /** Retain the working set, lift delta, and record undamped cost reduction.
     */
    void recordSolution(const IELMState &state,
                        const IndexSetMap &blockingIndices);

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
    double newError = std::numeric_limits<double>::infinity();
    double costChange = -std::numeric_limits<double>::infinity();
    size_t numRetractionIterations =
        0;  // total number of iterations in LM opt.
    std::vector<double> retractionDeviationRates;
    double linearCostChangeWithRetractionDelta = 0;

    /** Default constructor. */
    NonlinearUpdate() {}

    /** Compute the new manifolds using the linear update delta and blocking
     * indices. */
    NonlinearUpdate(const IELMState &state, const LinearUpdate &linearUpdate,
                    const NonlinearFactorGraph &graph,
                    size_t retractionThreads = 1);

    NonlinearUpdate(const IELMState &state, const IndexSetMap &forcedIndicesMap,
                    const NonlinearFactorGraph &graph);

    void computeError(const NonlinearFactorGraph &graph,
                      const double &oldError);

    static std::pair<VectorValues, double> evaluateRetractionDeviation(
        const IEConstraintManifold &manifold,
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
  double modelFidelity = -std::numeric_limits<double>::infinity();
  bool stepIsSuccessful = false;
  bool stopSearchingLambda = false;
  double trialTime = 0;

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

}  // namespace gtdynamics
