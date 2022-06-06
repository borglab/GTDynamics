/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizerType1.h
 * @brief Manifold optimizer that use a constraint manifold to represent each
 * constraint-connected-component.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/ManifoldOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

#include <boost/variant.hpp>

namespace gtsam {

/** Manifold Optimizer that replace each constraint-connected component with a
 * constraint manifold variable */
class ManifoldOptimizerType1 : public ManifoldOptimizer {
 public:
  using shared_ptr = boost::shared_ptr<const ManifoldOptimizerType1>;
  typedef boost::variant<GaussNewtonParams, LevenbergMarquardtParams,
                         DoglegParams>
      NonlinearOptParamsVariant;

 protected:
  boost::shared_ptr<NonlinearOptimizer> nonlinear_optimizer_;
  KeyVector component_key_vec_;
  KeySet unconstrained_keys_;
  Values fc_manifolds_;
  Values base_values_;

 public:
  /** Default constructor. */
  ManifoldOptimizerType1() {}

  /** Constructor. */
  ManifoldOptimizerType1(
      const gtsam::NonlinearFactorGraph& costs,
      const gtdynamics::EqualityConstraints& constraints,
      const gtsam::Values& init_values,
      const NonlinearOptParamsVariant& nopt_params,
      const ManifoldOptimizer::Params::shared_ptr& params =
          boost::make_shared<Params>(),
      boost::optional<BasisKeyFunc> basis_key_func = boost::none)
      : ManifoldOptimizer(costs, constraints, params, basis_key_func) {
    construct_nonlinear_optimizer(init_values, nopt_params);
  }

  /** Virtual destructor. */
  virtual ~ManifoldOptimizerType1() {}

  virtual const gtsam::Values& optimize() override;

  /// construct base values and return by reference
  const Values& base_values();

  int getInnerIterations() const;

  /** Call iterate() of NonlinearOptimizer. */
  void iterate() { nonlinear_optimizer_->iterate(); }

  virtual void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Dimension of the manifold optimization problem, as factor dimension x
   * variable dimension. */
  std::pair<size_t, size_t> problem_dimension() const;

 protected:
  /** Create the underlying nonlinear optimizer for manifold optimization. */
  void construct_nonlinear_optimizer(const Values& init_values,
                                     const NonlinearOptParamsVariant& params);

  /** Create initial values for the constraint manifold variables. */
  Values construct_manifold_values(const Values& init_values);

  /** Create a factor graph of cost function with the constraint manifold
   * variables. */
  NonlinearFactorGraph construct_manifold_graph(const Values& manifold_values);
};

}  // namespace gtsam
