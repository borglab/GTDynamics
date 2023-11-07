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

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

struct LMState;
struct LMTrial;
struct LMIterDetails;

/** State corresponding to each LM iteration. */
struct LMState {
public:
  EManifoldValues manifolds;
  Values unconstrained_values;
  EManifoldValues const_manifolds;
  double error = 0;
  double lambda = 0;
  double lambda_factor = 0;
  size_t iterations = 0;
  size_t totalNumberInnerIterations = 0;

  LMState() {}

  /// Constructor.
  LMState(const NonlinearFactorGraph &graph, const ManifoldOptProblem &problem,
          const double &_lambda, const double &_lambda_factor,
          size_t _iterations = 0);

  /// Initialize the new state from the result of previous iteration.
  static LMState FromLastIteration(const LMIterDetails &iter_details,
                                   const NonlinearFactorGraph &graph,
                                   const LevenbergMarquardtParams &params);

  static double EvaluateGraphError(const NonlinearFactorGraph &graph,
                                   const EManifoldValues &_manifolds,
                                   const Values &_unconstrained_values,
                                   const EManifoldValues &_const_manifolds);

  Values baseValues() const;
};

/** Trial for LM inner iteration with certain lambda setting. */
struct LMTrial {
  struct LinearUpdate;
  struct NonlinearUpdate;

  /// Perform a trial of iteration with specified lambda value.
  LMTrial(const LMState &state, const NonlinearFactorGraph &graph,
          const NonlinearFactorGraph &manifold_graph, const double &lambda,
          const LevenbergMarquardtParams &params);

  struct LinearUpdate {
    double lambda;
    VectorValues delta;
    VectorValues tangent_vector;
    double old_error;
    double new_error;
    double cost_change;
    bool solve_successful;

    /** Default constructor. */
    LinearUpdate() {}

    /** The function computes:
     */
    LinearUpdate(const double &_lambda,
                 const NonlinearFactorGraph &manifold_graph,
                 const LMState &state, const LevenbergMarquardtParams &params);

  protected:
    /** Following functions are adapted from LMOptimizerState. */

    // Small cache of A|b|model indexed by dimension. Can save many mallocs.
    mutable std::vector<LMCachedModel> noiseModelCache;

    LMCachedModel *getCachedModel(size_t dim) const;

    /// Build a damped system for a specific lambda, vanilla version
    GaussianFactorGraph
    buildDampedSystem(GaussianFactorGraph damped /* gets copied */,
                      const LMState &state) const;

    /// Build a damped system, use hessianDiagonal per variable (more expensive)
    GaussianFactorGraph
    buildDampedSystem(GaussianFactorGraph damped, // gets copied
                      const VectorValues &sqrtHessianDiagonal) const;

    GaussianFactorGraph
    buildDampedSystem(const GaussianFactorGraph &linear,
                      const VectorValues &sqrtHessianDiagonal,
                      const LMState &state,
                      const LevenbergMarquardtParams &params) const;

    VectorValues computeTangentVector(const VectorValues &delta,
                                      const LMState &state) const;
  };

  struct NonlinearUpdate {
    EManifoldValues new_manifolds;
    Values new_unconstrained_values;
    double new_error;
    double cost_change;

    /** Default constructor. */
    NonlinearUpdate() {}

    /** Compute the new manifolds using the linear update delta and blocking
     * indices. */
    NonlinearUpdate(const LMState &state, const LinearUpdate &linear_update,
                    const NonlinearFactorGraph &graph);
  };

public:
  // linear update
  LinearUpdate linear_update;

  // nonlinear update
  NonlinearUpdate nonlinear_update;

  // decision making
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

  /** Print summary info of the trial. */
  void print(const LMState &state) const;

  static void PrintTitle();
};

struct LMIterDetails {
  LMState state;
  std::vector<LMTrial> trials;

  LMIterDetails(const LMState &_state) : state(_state), trials() {}
};

class LMItersDetails : public std::vector<LMIterDetails> {
public:
  using base = std::vector<LMIterDetails>;
  using base::base;

  void exportFile(const std::string &state_file_path,
                  const std::string &trial_file_path) const;
};

} // namespace gtsam
