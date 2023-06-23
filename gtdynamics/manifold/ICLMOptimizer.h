/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ICLMOptimizer.h
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#pragma once

#include <chrono>
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

struct ICOptimizerState {
public:
  std::vector<IneqConstraintManifold::shared_ptr> manifolds;
  Values values;
  double error;
  size_t iterations;
  double lambda;
  double currentFactor = 0;
  int totalNumberInnerIterations = 0;

  ICOptimizerState(
      const std::vector<IneqConstraintManifold::shared_ptr> &_manifolds,
      const NonlinearFactorGraph &graph, size_t _iterations = 0)
      : manifolds(_manifolds), values(CollectManifoldValues(manifolds)),
        error(graph.error(values)), iterations(_iterations) {}

  void increaseLambda(const LevenbergMarquardtParams& params) {
    lambda *= currentFactor;
    totalNumberInnerIterations += 1;
    if (!params.useFixedLambdaFactor) {
      currentFactor *= 2.0;
    }
  }

  // Apply policy to decrease lambda if the current update was successful
  // stepQuality not used in the naive policy)
  // Take ownsership of newValues, must be passed an rvalue
  void decreaseLambda(const LevenbergMarquardtParams& params, double stepQuality) {
    double newLambda = lambda, newFactor = currentFactor;
    if (params.useFixedLambdaFactor) {
      newLambda /= currentFactor;
    } else {
      newLambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * stepQuality - 1.0, 3));
      newFactor = 2.0 * currentFactor;
    }
    lambda = std::max(params.lambdaLowerBound, newLambda);
    totalNumberInnerIterations += 1;
    currentFactor = newFactor;
  }

  struct CachedModel {
    CachedModel() {}  // default int makes zero-size matrices
    CachedModel(int dim, double sigma)
        : A(Matrix::Identity(dim, dim)),
          b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    CachedModel(int dim, double sigma, const Vector& diagonal)
        : A(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal)),
          b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    Matrix A;
    Vector b;
    SharedDiagonal model;
  };

  mutable std::vector<CachedModel> noiseModelCache;
  CachedModel* getCachedModel(size_t dim) const {
    if (dim >= noiseModelCache.size())
      noiseModelCache.resize(dim+1);
    CachedModel* item = &noiseModelCache[dim];
    if (!item->model)
      *item = CachedModel(dim, 1.0 / std::sqrt(lambda));
    return item;
  }

  /// Build a damped system for a specific lambda, vanilla version
  GaussianFactorGraph buildDampedSystem(GaussianFactorGraph damped /* gets copied */) const {
    noiseModelCache.resize(0);
    // for each of the variables, add a prior
    damped.reserve(damped.size() + values.size());
    std::map<Key,size_t> dims = values.dims();
    for (const auto& key_dim : dims) {
      const Key& key = key_dim.first;
      const size_t& dim = key_dim.second;
      const CachedModel* item = getCachedModel(dim);
      damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
    }
    return damped;
  }

  /// Build a damped system, use hessianDiagonal per variable (more expensive)
  GaussianFactorGraph buildDampedSystem(GaussianFactorGraph damped,  // gets copied
                                        const VectorValues& sqrtHessianDiagonal) const {
    noiseModelCache.resize(0);
    damped.reserve(damped.size() + values.size());
    for (const auto& key_vector : sqrtHessianDiagonal) {
      try {
        const Key key = key_vector.first;
        const size_t dim = key_vector.second.size();
        CachedModel* item = getCachedModel(dim);
        item->A.diagonal() = sqrtHessianDiagonal.at(key);  // use diag(hessian)
        damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
      } catch (const std::out_of_range&) {
        continue;  // Don't attempt any damping if no key found in diagonal
      }
    }
    return damped;
  }

};

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class ICLMOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters

public:
  typedef std::shared_ptr<ICLMOptimizer> shared_ptr;

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param initialValues The initial variable assignments
   * @param params The optimization parameters
   */
  ICLMOptimizer(
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams()) {}

  /** Virtual destructor */
  ~ICLMOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  Values optimize(const NonlinearFactorGraph &graph,
                  const std::vector<IneqConstraintManifold::shared_ptr>
                      &ic_manifolds,
                  gtdynamics::ConstrainedOptResult* intermediate_result = nullptr) const;

  bool checkConvergence(double relativeErrorTreshold,
                        double absoluteErrorTreshold, double errorThreshold,
                        double currentError, double newError) const;

  ICOptimizerState iterate(const NonlinearFactorGraph &graph,
                           const ICOptimizerState &state) const;

  /// linearized graph with tight equality constraints
  virtual GaussianFactorGraph::shared_ptr
  linearize(const NonlinearFactorGraph &graph,
            const ICOptimizerState &state) const;

  /** Inner loop, changes state, returns true if successful or giving up */
  bool tryLambda(const NonlinearFactorGraph &graph,
                 const GaussianFactorGraph &linear,
                 const VectorValues &sqrtHessianDiagonal,
                 ICOptimizerState &state) const;

  /** Build a damped system for a specific lambda -- for testing only */
  GaussianFactorGraph
  buildDampedSystem(const ICOptimizerState& state,
                    const GaussianFactorGraph &linear,
                    const VectorValues &sqrtHessianDiagonal) const;

  virtual VectorValues solve(const GaussianFactorGraph &gfg,
                             const NonlinearOptimizerParams &params) const;

  std::vector<IneqConstraintManifold::shared_ptr>
  retract_manifolds(const ICOptimizerState &state,
                    const VectorValues &delta) const;
};

} // namespace gtsam
