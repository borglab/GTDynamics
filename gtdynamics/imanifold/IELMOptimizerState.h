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

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>

namespace gtsam {

struct IELMState;
struct IELMTrial;
struct IELMIterDetails;
struct IELMParams;

/** State corresponding to each LM iteration. */
struct IELMState {
public:
  IEManifoldValues manifolds;
  Values values;
  KeySet unconstrained_keys;
  double error = 0;
  double lambda = 0;
  double lambda_factor = 0;
  size_t iterations = 0;
  size_t totalNumberInnerIterations = 0;
  VectorValues gradient;
  IndexSetMap grad_blocking_indices_map; // blocking indices map by neg grad
  GaussianFactorGraph::shared_ptr base_linear;
  GaussianFactorGraph::shared_ptr linear_manifold_graph;
  gtdynamics::LinearInequalityConstraints linear_base_i_constraints;
  gtdynamics::LinearInequalityConstraints linear_manifold_i_constraints;
  gtsam::IndexSetMapTranslator lic_index_translator;

  /// Default constructor.
  IELMState() {}

  /// Constructor.
  IELMState(const IEManifoldValues &_manifolds,
            const Values &_unconstrained_values,
            const NonlinearFactorGraph &graph,
            const NonlinearFactorGraph &manifold_graph, const double &_lambda,
            const double &_lambda_factor, size_t _iterations = 0);

  /// Initialize the new state from the result of previous iteration.
  static IELMState FromLastIteration(const IELMIterDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const NonlinearFactorGraph &manifold_graph,
                                     const LevenbergMarquardtParams &params);

  /// Values of unconstrained variables.
  Values unconstrainedValues() const;

  /// Values of all original constrained opt problem variables.
  Values baseValues() const;

  static double EvaluateGraphError(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &_manifolds,
                                   const Values &_unconstrained_values);

  /// Compute the hessian diagonal of the linearized graph. It is used to
  /// construct metric sigma for projection.
  VectorValues computeMetricSigmas(const NonlinearFactorGraph &graph) const;

protected:
  static Values AllValues(const IEManifoldValues &manifolds,
                          const Values &unconstrained_values);

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
            const IndexSetMap &forced_indices_map);

  struct LinearUpdate {
    double lambda;
    IndexSetMap blocking_indices_map;
    VectorValues delta;
    VectorValues tangent_vector;
    double old_error;
    double new_error;
    double cost_change;
    bool solve_successful;
    size_t num_solves; // number of solving linear systems

    /** Default constructor. */
    LinearUpdate() {}

    /** The function computes:
     * 1) linear update as (approx) solution the i-constrained qp problem
     * 2) blocking constraints as of solving the IQP problem
     * 3) e-manifolds corresponding to the e-constraints and blocking
     * constraints 4) corresponding linear cost change 5) solve_successful
     * indicating if solving the linear system is successful
     */
    LinearUpdate(const double &_lambda, const NonlinearFactorGraph &graph,
                 const IELMState &state,
                 const IELMParams &params);

    /** Generate an initial estimate for the IQP problem using active
     * constraints that include the blocking constraint for neg-gradient.
     * @return [delta, blocking_indices, num_solves, solve_sucessful]
     */
    static std::tuple<VectorValues, IndexSet, size_t, bool>
    InitEstimate(const GaussianFactorGraph &quadratic_cost,
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
     * @param tangent_vector Solution to the IQP problem.
     * @param blocking_indices Indices of constraints that are active in IQP
     * w.r.t. delta.
     */
    static bool CheckSolutionValid(const IELMState &state,
                                   const VectorValues &tangent_vector,
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
                                      const KeySet &unconstrained_keys) const;
  };

  struct NonlinearUpdate {
    IEManifoldValues new_manifolds;
    Values new_unconstrained_values;
    double new_error;
    double cost_change;
    size_t num_retract_iters; // total number of iterations in LM opt.
    std::vector<double> retract_divate_rates;
    double linear_cost_change_with_retract_delta;

    /** Default constructor. */
    NonlinearUpdate() {}

    /** Compute the new manifolds using the linear update delta and blocking
     * indices. */
    NonlinearUpdate(const IELMState &state, const LinearUpdate &linear_update,
                    const NonlinearFactorGraph &graph);

    void computeError(const NonlinearFactorGraph &graph,
                      const double &old_error);
  };

public:
  // linear update
  LinearUpdate linear_update;

  // nonlinear update
  NonlinearUpdate nonlinear_update;

  // decision making
  IndexSetMap forced_indices_map;
  double model_fidelity;
  bool step_is_successful;
  bool stop_searching_lambda;
  double trial_time;

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_lambda, double &new_lambda_factor,
                     const LevenbergMarquardtParams &params) const;

  /// Set lambda as increased values for next trial/state.
  void setIncreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;

  /// Set lambda as decreased values for next trial/state.
  void setDecreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;
};

/** Print title of summary info of IELM trials. */
void PrintIELMTrialTitle();

/** Print summary info of an IELM trial. */
void PrintIELMTrial(
    const IELMState &state, const IELMTrial &trial, const IELMParams &params,
    bool forced = false,
    const KeyFormatter &key_formatter = gtdynamics::GTDKeyFormatter);

struct IELMIterDetails {
  IELMState state;
  std::vector<IELMTrial> trials;

  IELMIterDetails(const IELMState &_state) : state(_state), trials() {}
};

class IELMItersDetails : public std::vector<IELMIterDetails> {
public:
  using base = std::vector<IELMIterDetails>;
  using base::base;

  void exportFile(const std::string &state_file_path,
                  const std::string &trial_file_path) const;
};

} // namespace gtsam
