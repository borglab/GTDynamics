/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizer.h
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
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

struct IELMNonlinearIterDetails {
  IEManifoldValues manifolds;
  std::map<Key, IndexSet> blocking_indices_map;
  std::map<Key, IndexSet> forced_indices_map;
  VectorValues delta;
  IEManifoldValues new_manifolds;
  double lambda;
  double cost;
  double new_error;
  double linear_cost_change;
  double nonlinear_cost_change;
  double model_fidelity;
  bool solve_successful;
  bool step_is_successful;
  bool stop_searching_lambda;
};

struct IELMIterDetails {
  IELMNonlinearIterDetails initial;
  std::vector<IELMNonlinearIterDetails> trials;

  IELMIterDetails(const IELMNonlinearIterDetails &_initial)
      : initial(_initial), trials() {}
};

struct IELMOptimizerState {
public:
  IEManifoldValues manifolds;
  std::map<Key, IndexSet> blocking_indices_map;
  Values e_manifolds;
  Values const_e_manifolds;
  double error;
  size_t iterations;
  double lambda;
  double currentFactor = 0;
  int totalNumberInnerIterations = 0;

  IELMOptimizerState(const IEManifoldValues &_manifolds,
                     const NonlinearFactorGraph &graph,
                     size_t _iterations = 0)
      : manifolds(_manifolds),
        error(graph.error(IEOptimizer::CollectManifoldValues(manifolds))), iterations(_iterations) {
    ConstructEManifolds(graph);
  }

  void updateManifolds(const IEManifoldValues &_manifolds,
                       const NonlinearFactorGraph &graph) {
    manifolds = _manifolds;
    error = graph.error(IEOptimizer::CollectManifoldValues(manifolds));
    ConstructEManifolds(graph);
  }

  std::map<Key, IndexSet> identifyBlockingIndices(const NonlinearFactorGraph &manifold_graph) const {
    Values e_bare_manifolds = IEOptimizer::EManifolds(manifolds);
    auto linear_graph = manifold_graph.linearize(e_bare_manifolds);
    VectorValues descent_dir = -1 * linear_graph->gradientAtZero();

    // identify blocking constraints
    return IEOptimizer::ProjectTangentCone(manifolds, descent_dir).first;
  }

  void ConstructEManifolds(const NonlinearFactorGraph &graph) {
    std::map<Key, Key> keymap_var2manifold = IEOptimizer::Var2ManifoldKeyMap(manifolds);
    NonlinearFactorGraph manifold_graph = ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);
    blocking_indices_map = identifyBlockingIndices(manifold_graph);
    // setting blocking constraints as equalities, create e-manifolds
    std::tie(e_manifolds, const_e_manifolds) = IEOptimizer::EManifolds(manifolds, blocking_indices_map);
  }

  void increaseLambda(const LevenbergMarquardtParams &params) {
    lambda *= currentFactor;
    totalNumberInnerIterations += 1;
    if (!params.useFixedLambdaFactor) {
      currentFactor *= 2.0;
    }
  }

  // Apply policy to decrease lambda if the current update was successful
  // stepQuality not used in the naive policy)
  // Take ownsership of newValues, must be passed an rvalue
  void decreaseLambda(const LevenbergMarquardtParams &params,
                      double stepQuality) {
    double newLambda = lambda, newFactor = currentFactor;
    if (params.useFixedLambdaFactor) {
      newLambda /= currentFactor;
    } else {
      newLambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * stepQuality - 1.0, 3));
      newFactor = 2.0 * currentFactor;
    }
    lambda = std::max(params.lambdaLowerBound, newLambda);
    iterations += 1;
    totalNumberInnerIterations += 1;
    currentFactor = newFactor;
  }

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
  CachedModel *getCachedModel(size_t dim) const {
    if (dim >= noiseModelCache.size())
      noiseModelCache.resize(dim + 1);
    CachedModel *item = &noiseModelCache[dim];
    if (!item->model)
      *item = CachedModel(dim, 1.0 / std::sqrt(lambda));
    return item;
  }

  /// Build a damped system for a specific lambda, vanilla version
  GaussianFactorGraph
  buildDampedSystem(GaussianFactorGraph damped /* gets copied */) const {
    noiseModelCache.resize(0);
    // for each of the variables, add a prior
    damped.reserve(damped.size() + e_manifolds.size());
    std::map<Key, size_t> dims = e_manifolds.dims();
    for (const auto &key_dim : dims) {
      const Key &key = key_dim.first;
      const size_t &dim = key_dim.second;
      const CachedModel *item = getCachedModel(dim);
      damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
    }
    return damped;
  }

  /// Build a damped system, use hessianDiagonal per variable (more expensive)
  GaussianFactorGraph
  buildDampedSystem(GaussianFactorGraph damped, // gets copied
                    const VectorValues &sqrtHessianDiagonal) const {
    noiseModelCache.resize(0);
    damped.reserve(damped.size() + e_manifolds.size());
    for (const auto &key_vector : sqrtHessianDiagonal) {
      try {
        const Key key = key_vector.first;
        const size_t dim = key_vector.second.size();
        CachedModel *item = getCachedModel(dim);
        item->A.diagonal() = sqrtHessianDiagonal.at(key); // use diag(hessian)
        damped.emplace_shared<JacobianFactor>(key, item->A, item->b,
                                              item->model);
      } catch (const std::out_of_range &) {
        continue; // Don't attempt any damping if no key found in diagonal
      }
    }
    return damped;
  }

  VectorValues computeTangentVector(const VectorValues &delta) const {
    VectorValues tangent_vector;
    for (const Key &key : e_manifolds.keys()) {
      const Vector &xi = delta.at(key);
      ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
      VectorValues tv = e_manifold.basis()->computeTangentVector(xi);
      tangent_vector.insert(tv);
    }
    for (const Key &key : const_e_manifolds.keys()) {
      ConstraintManifold e_manifold = const_e_manifolds.at<ConstraintManifold>(key);
      VectorValues tv = e_manifold.values().zeroVectors();
      tangent_vector.insert(tv);
    }
    return tangent_vector;
  }

  IEManifoldValues retractManifolds(const VectorValues &delta,
                                    const std::optional<std::map<Key, IndexSet>>
                                        &blocking_indices_map = {}) const {
    IEManifoldValues new_manifolds;
    for (const auto &it : manifolds) {
      const Key &key = it.first;
      if (const_e_manifolds.exists(key)) {
        new_manifolds.emplace(key, it.second);
      }
      else {
        const Vector &xi = delta.at(key);
        ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
        VectorValues tv = e_manifold.basis()->computeTangentVector(xi);
        if (blocking_indices_map) {
          const auto &blocking_indices = blocking_indices_map->at(key);
          new_manifolds.emplace(key, it.second.retract(tv, blocking_indices));
        } else {
          new_manifolds.emplace(key, it.second.retract(tv));
        }
      }
    }
    return new_manifolds;
  }
};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class IELMOptimizer : public IEOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters
  std::shared_ptr<std::vector<IELMIterDetails>> details_;

public:
  typedef std::shared_ptr<IELMOptimizer> shared_ptr;

  const std::vector<IELMIterDetails>& details() const {return *details_; }

  /** Constructor */
  IELMOptimizer(
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : IEOptimizer(), params_(params), details_(std::make_shared<std::vector<IELMIterDetails>>()) {}

  /** Virtual destructor */
  ~IELMOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  Values optimizeManifolds(const NonlinearFactorGraph &graph,
                           const IEManifoldValues &manifolds,
                           gtdynamics::ConstrainedOptResult
                               *intermediate_result = nullptr) const override;

  bool checkConvergence(double relativeErrorTreshold,
                        double absoluteErrorTreshold, double errorThreshold,
                        double currentError, double newError) const;

  IELMOptimizerState iterate(const NonlinearFactorGraph &graph,
                             const IELMOptimizerState &state,
                             gtdynamics::ConstrainedOptResult
                               *intermediate_result = nullptr) const;

  /// linearized graph with tight equality constraints
  virtual GaussianFactorGraph::shared_ptr
  linearize(const NonlinearFactorGraph &graph,
            const IELMOptimizerState &state) const;

  /** Inner loop, changes state, returns true if successful or giving up */
  IELMNonlinearIterDetails tryLambda(const NonlinearFactorGraph &graph,
                 const GaussianFactorGraph &linear,
                 const VectorValues &sqrtHessianDiagonal,
                 const IELMOptimizerState &state) const;

  /** Build a damped system for a specific lambda -- for testing only */
  GaussianFactorGraph
  buildDampedSystem(const IELMOptimizerState &state,
                    const GaussianFactorGraph &linear,
                    const VectorValues &sqrtHessianDiagonal) const;

  virtual VectorValues solve(const GaussianFactorGraph &gfg,
                             const NonlinearOptimizerParams &params) const;

  bool checkModeChange(IELMOptimizerState& state, const NonlinearFactorGraph& graph, IELMNonlinearIterDetails& iter_details) const;

};

} // namespace gtsam
