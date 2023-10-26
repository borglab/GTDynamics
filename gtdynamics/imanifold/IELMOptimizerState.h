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
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

struct IELMState;
struct IELMTrial;
struct IELMIterDetails;

/** State corresponding to each LM iteration. */
struct IELMState {
public:
  IEManifoldValues manifolds;
  double error = 0;
  VectorValues gradient;
  IndexSetMap blocking_indices_map; // blocking indices map by neg grad
  Values e_manifolds;
  Values const_e_manifolds;
  double lambda = 0;
  double lambda_factor = 0;
  size_t iterations = 0;
  size_t totalNumberInnerIterations = 0;

  IELMState() {}

  /// Constructor.
  IELMState(const IEManifoldValues &_manifolds,
            const NonlinearFactorGraph &graph, size_t _iterations = 0)
      : manifolds(_manifolds),
        error(graph.error(IEOptimizer::CollectManifoldValues(manifolds))),
        iterations(_iterations) {
    ConstructEManifolds(graph);
  }

  /// Initialize the new state from the result of previous iteration.
  static IELMState FromLastIteration(const IELMIterDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const LevenbergMarquardtParams &params);

  /// Identify the blocking constraints using gradient.
  void identifyGradBlockingIndices(const NonlinearFactorGraph &manifold_graph);

  /// Construct e-manifolds using grad blocking constraints.
  void ConstructEManifolds(const NonlinearFactorGraph &graph);
};

/** Trial for LM inner iteration with certain lambda setting. */
struct IELMTrial {
  IndexSetMap forced_indices_map;
  IndexSetMap blocking_indices_map;
  Values e_manifolds;
  Values const_e_manifolds;
  VectorValues delta;
  VectorValues tangent_vector;
  double old_linear_error;
  double new_linear_error;
  double linear_cost_change;
  double new_error;
  double nonlinear_cost_change;
  double model_fidelity;
  double trial_time;
  bool solve_successful;
  bool step_is_successful;
  bool stop_searching_lambda;
  IEManifoldValues new_manifolds;

  double lambda;
  double lambda_factor;

  /// Set lambda values same as the state, used for first trial
  void setLambda(const IELMState &state);

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_lambda, double &new_lambda_factor,
                     const LevenbergMarquardtParams &params) const;

  /// Set lambda as increased values for next trial/state.
  void setIncreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;

  /// Set lambda as decreased values for next trial/state.
  void setDecreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;

  /// Linearize based on the e-manifolds.
  GaussianFactorGraph::shared_ptr
  linearize(const NonlinearFactorGraph &graph,
            const std::map<Key, Key> &keymap_var2manifold) const;

  /// Solve gaussian factor graph using specified parameters.
  static VectorValues solve(const GaussianFactorGraph &gfg,
                            const NonlinearOptimizerParams &params);

  /** The function computes:
   * 1) linear update as (approx) solution the i-constrained qp problem
   * 2) blocking constraints as of solving the IQP problem
   * 3) e-manifolds corresponding to the e-constraints and blocking constraints
   * 4) corresponding linear cost change
   * 5) solve_successful indicating if solving the linear system is successful
   */
  bool computeDelta(const NonlinearFactorGraph &graph, const IELMState &state,
                    const LevenbergMarquardtParams &params);

  /** Compute the new manifolds using the linear update delta and blocking
   * indices. */
  void computeNewManifolds(const IELMState &state);

  /** Compute tangent vector from delta. */
  void computeTangentVector();

  /** Print summary info of the trial. */
  void print(const IELMState &state) const;

  /** Following functions are adapted from LMOptimizerState. */
  struct CachedModel {
    CachedModel() {} // default int makes zero-size matrices
    CachedModel(int dim, double sigma)
        : A(Matrix::Identity(dim, dim)), b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    CachedModel(int dim, double sigma, const Vector &diagonal)
        : A(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal)),
          b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    Matrix A;
    Vector b;
    SharedDiagonal model;
  };

  mutable std::vector<CachedModel> noiseModelCache;
  CachedModel *getCachedModel(size_t dim) const;

  GaussianFactorGraph
  buildDampedSystem(GaussianFactorGraph damped /* gets copied */) const;

  GaussianFactorGraph
  buildDampedSystem(GaussianFactorGraph damped, // gets copied
                    const VectorValues &sqrtHessianDiagonal) const;

  GaussianFactorGraph
  buildDampedSystem(const GaussianFactorGraph &linear,
                    const VectorValues &sqrtHessianDiagonal,
                    const LevenbergMarquardtParams &params) const;
};

struct IELMIterDetails {
  IELMState state;
  std::vector<IELMTrial> trials;

  IELMIterDetails(const IELMState &_state) : state(_state), trials() {}
};

typedef std::vector<IELMIterDetails> IELMItersDetails;

} // namespace gtsam