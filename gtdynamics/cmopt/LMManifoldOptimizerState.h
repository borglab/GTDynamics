/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LMManifoldOptimizerState.h
 * @brief   State and Trial for LMManifoldOptimizer
 * @author  Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtdynamics {

using gtsam::GaussianFactorGraph;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using gtsam::VectorValues;

struct LMState;
struct LMTrial;
struct LMIterationDetails;

/**
 * State for one outer iteration of manifold LM optimization.
 *
 * This structure stores manifold variables, unconstrained variables, fixed
 * manifold variables, and LM damping/error metadata used by trial generation
 * and convergence checks.
 *
 * @see README.md#solvers
 */
struct LMState {
 public:
  EManifoldValues manifolds;
  Values unconstrainedValues;
  EManifoldValues constManifolds;
  double error = 0;
  double lambda = 0;
  double lambdaFactor = 0;
  size_t iterations = 0;
  size_t totalInnerIterations = 0;

  LMState() {}

  /**
   * Constructor.
   * @param graph Base cost graph.
   * @param problem Transformed manifold optimization problem.
   * @param _lambda Initial damping value.
   * @param _lambdaFactor Lambda scaling factor.
   * @param _iterations Current outer iteration index.
   */
  LMState(const NonlinearFactorGraph &graph, const ManifoldOptimizationProblem &problem,
          const double &_lambda, const double &_lambdaFactor,
          size_t _iterations = 0);

  /**
   * Initialize the next state from the previous iteration details.
   * @param iter_details Completed iteration details.
   * @param graph Base cost graph.
   * @param params LM parameters.
   * @return New LM state for the next outer iteration.
   */
  static LMState fromLastIteration(const LMIterationDetails &iter_details,
                                   const NonlinearFactorGraph &graph,
                                   const LevenbergMarquardtParams &params);

  /**
   * Evaluate base graph error from manifold and unconstrained values.
   * @param graph Base cost graph.
   * @param _manifolds Current manifold values.
   * @param _unconstrained_values Current unconstrained values.
   * @param _const_manifolds Fixed manifold values.
   * @return Total nonlinear graph error.
   */
  static double evaluateGraphError(const NonlinearFactorGraph &graph,
                                   const EManifoldValues &_manifolds,
                                   const Values &_unconstrained_values,
                                   const EManifoldValues &_const_manifolds);

  /// Reconstruct base variable values from state partitions.
  Values baseValues() const;
};

/**
 * One inner LM trial at a specific lambda.
 *
 * Each trial computes a linearized update and a nonlinear update, evaluates
 * model fidelity, and decides whether the trial is accepted.
 *
 * @see README.md#solvers
 */
struct LMTrial {
  struct LinearUpdate;
  struct NonlinearUpdate;

  /**
   * Perform a single LM trial at a specified lambda.
   * @param state Current LM state.
   * @param graph Base cost graph.
   * @param manifold_graph Transformed manifold graph.
   * @param lambda Trial lambda.
   * @param params LM parameters.
   */
  LMTrial(const LMState &state, const NonlinearFactorGraph &graph,
          const NonlinearFactorGraph &manifold_graph, const double &lambda,
          const LevenbergMarquardtParams &params);

  /**
   * Linearized LM update for a single trial.
   *
   * Stores the solved delta, lifted tangent vector, linearized error change,
   * and solve success flag.
   *
   * @see README.md#solvers
   */
  struct LinearUpdate {
    double lambda;
    VectorValues delta;
    VectorValues tangentVector;
    double oldError;
    double newError;
    double costChange;
    bool solveSuccessful;

    /** Default constructor. */
    LinearUpdate() {}

    /**
     * Compute the linearized LM update for one trial.
     * @param _lambda Trial lambda.
     * @param manifold_graph Transformed manifold graph.
     * @param state Current LM state.
     * @param params LM parameters.
     */
    LinearUpdate(const double &_lambda,
                 const NonlinearFactorGraph &manifold_graph,
                 const LMState &state, const LevenbergMarquardtParams &params);

   protected:
    /** Following functions are adapted from LMOptimizerState. */

    // Small cache of A|b|model indexed by dimension. Can save many mallocs.
    mutable std::vector<LMCachedModel> noiseModelCache;

    LMCachedModel *getCachedModel(size_t dim) const;

    /**
     * Build a damped linear system with isotropic damping.
     * @param damped Linearized graph copy.
     * @param state Current LM state.
     * @return Damped Gaussian factor graph.
     */
    GaussianFactorGraph buildDampedSystem(
        GaussianFactorGraph damped /* gets copied */,
        const LMState &state) const;

    /**
     * Build a damped linear system using Hessian-diagonal damping.
     * @param damped Linearized graph copy.
     * @param sqrtHessianDiagonal Square roots of Hessian diagonal blocks.
     * @return Damped Gaussian factor graph.
     */
    GaussianFactorGraph buildDampedSystem(
        GaussianFactorGraph damped,  // gets copied
        const VectorValues &sqrtHessianDiagonal) const;

    /**
     * Dispatch damped-system construction based on LM damping configuration.
     * @param linear Linearized graph.
     * @param sqrtHessianDiagonal Square roots of Hessian diagonal blocks.
     * @param state Current LM state.
     * @param params LM parameters.
     * @return Damped Gaussian factor graph.
     */
    GaussianFactorGraph buildDampedSystem(
        const GaussianFactorGraph &linear,
        const VectorValues &sqrtHessianDiagonal, const LMState &state,
        const LevenbergMarquardtParams &params) const;

    /**
     * Lift manifold-space delta to base-space tangent vectors.
     * @param delta Delta in transformed variable space.
     * @param state Current LM state.
     * @return Tangent vectors for base variables.
     */
    VectorValues computeTangentVector(const VectorValues &delta,
                                      const LMState &state) const;
  };

  /**
   * Nonlinear update obtained by retracting a linear trial update.
   *
   * Stores updated manifold/unconstrained values and nonlinear error change.
   *
   * @see README.md#solvers
   */
  struct NonlinearUpdate {
    EManifoldValues newManifolds;
    Values newUnconstrainedValues;
    double newError;
    double costChange;

    /** Default constructor. */
    NonlinearUpdate() {}

    /**
     * Compute nonlinear trial update from a linear update.
     * @param state Current LM state.
     * @param linearUpdate Linear trial update.
     * @param graph Base cost graph.
     */
    NonlinearUpdate(const LMState &state, const LinearUpdate &linearUpdate,
                    const NonlinearFactorGraph &graph);
  };

 public:
  // linear update
  LinearUpdate linearUpdate;

  // nonlinear update
  NonlinearUpdate nonlinearUpdate;

  // decision making
  double modelFidelity;
  bool stepIsSuccessful;
  bool stopSearchingLambda;
  double trialTime;

  /**
   * Update lambda for the next trial/state.
   * @param new_lambda Output lambda value.
   * @param newLambdaFactor Output lambda scaling factor.
   * @param params LM parameters.
   */
  void setNextLambda(double &new_lambda, double &newLambdaFactor,
                     const LevenbergMarquardtParams &params) const;

  /**
   * Increase lambda for the next trial/state.
   * @param new_lambda Output lambda value.
   * @param newLambdaFactor Output lambda scaling factor.
   * @param params LM parameters.
   */
  void setIncreasedNextLambda(double &new_lambda, double &newLambdaFactor,
                              const LevenbergMarquardtParams &params) const;

  /**
   * Decrease lambda for the next trial/state.
   * @param new_lambda Output lambda value.
   * @param newLambdaFactor Output lambda scaling factor.
   * @param params LM parameters.
   */
  void setDecreasedNextLambda(double &new_lambda, double &newLambdaFactor,
                              const LevenbergMarquardtParams &params) const;

  /**
   * Print summary information of this trial.
   * @param state State corresponding to the trial.
   */
  void print(const LMState &state) const;

  /// Print table header for LM trial summaries.
  static void printTitle();
};

/**
 * Aggregated details for one outer iteration, including all inner trials.
 *
 * @see README.md#solvers
 */
struct LMIterationDetails {
  LMState state;
  std::vector<LMTrial> trials;

  LMIterationDetails(const LMState &_state) : state(_state), trials() {}
};

/**
 * Sequence of iteration details for an entire optimization run.
 *
 * @see README.md#solvers
 */
class LMOptimizationDetails : public std::vector<LMIterationDetails> {
 public:
  using base = std::vector<LMIterationDetails>;
  using base::base;

  /**
   * Export iteration and trial details to CSV files.
   * @param state_file_path Output path for per-state CSV.
   * @param trial_file_path Output path for per-trial CSV.
   */
  void exportFile(const std::string &state_file_path,
                  const std::string &trial_file_path) const;
};

}  // namespace gtdynamics
