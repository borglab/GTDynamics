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
  Values unconstrained_values;
  double error = 0;
  double lambda = 0;
  double lambda_factor = 0;
  size_t iterations = 0;
  size_t totalNumberInnerIterations = 0;
  VectorValues gradient;
  IndexSetMap blocking_indices_map; // blocking indices map by neg grad
  Values e_manifolds;
  Values const_e_manifolds;
  GaussianFactorGraph::shared_ptr base_linear;

  IELMState() {}

  /// Constructor.
  IELMState(const IEManifoldValues &_manifolds,
            const Values &_unconstrained_values,
            const NonlinearFactorGraph &graph, const double &_lambda,
            const double &_lambda_factor, size_t _iterations = 0);

  /// Initialize the new state from the result of previous iteration.
  static IELMState FromLastIteration(const IELMIterDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const LevenbergMarquardtParams &params);

  Values baseValues() const;

  static double EvaluateGraphError(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &_manifolds,
                                   const Values &_unconstrained_values);

  /// Identify the blocking constraints using gradient.
  void identifyGradBlockingIndices(const NonlinearFactorGraph &manifold_graph);

  /// Construct e-manifolds using grad blocking constraints.
  void ConstructEManifolds(const NonlinearFactorGraph &graph);

  /// Compute the hessian diagonal of the linearized graph. It is used to
  /// construct metric sigma for projection.
  VectorValues computeMetricSigmas(const NonlinearFactorGraph &graph) const;
};

/** Trial for LM inner iteration with certain lambda setting. */
struct IELMTrial {
  struct LinearUpdate;
  struct NonlinearUpdate;

  /// Perform a trial of iteration with specified lambda value.
  IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
            const double &lambda, const LevenbergMarquardtParams &params);

  /// Perform a trial by moving to specified boundaries.
  IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
            const IndexSetMap &forced_indices_map);

  struct LinearUpdate {
    double lambda;
    IndexSetMap blocking_indices_map;
    Values e_manifolds;
    Values const_e_manifolds;
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
                 const LevenbergMarquardtParams &params);

  protected:
    /// Linearize based on the e-manifolds.
    GaussianFactorGraph::shared_ptr
    linearize(const NonlinearFactorGraph &graph,
              const Values &unconstrained_values,
              const std::map<Key, Key> &keymap_var2manifold) const;

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

    VectorValues
    computeTangentVector(const VectorValues &delta,
                         const KeyVector &unconstrained_keys) const;
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
