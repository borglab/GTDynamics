/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  manifold_opt_benchmark.h
 * @brief Helper functions for benchmarking constraint manifold optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/optimizer/ManifoldOptimizerType1.h>

using gtsam::LevenbergMarquardtParams, gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph, gtsam::Values;
using gtsam::ConstraintManifold, gtsam::ManifoldOptimizer;

namespace gtdynamics {

/** Kinematic trajectory optimization using dynamics factor graph. */
Values optimize_dynamics_graph(const NonlinearFactorGraph& constraints_graph,
                               const NonlinearFactorGraph& costs,
                               const Values& init_values) {
  NonlinearFactorGraph graph;
  graph.add(constraints_graph);
  graph.add(costs);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gttic_(dynamic_factor_graph);
  auto result = optimizer.optimize();
  gttoc_(dynamic_factor_graph);

  size_t graph_dim = 0;
  for (const auto& factor : graph) {
    graph_dim += factor->dim();
  }
  std::cout << "dimension: " << graph_dim << " x " << init_values.dim() << "\n";
  return result;
}

/** Run optimization using constraint manifold. */
Values optimize_constraint_manifold(const EqualityConstraints& constraints,
                                    const NonlinearFactorGraph& costs,
                                    const Values& init_values) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  // params.minModelFidelity = 0.1;
  std::cout << "building optimizer\n";
  gtsam::ManifoldOptimizerType1 optimizer(costs, constraints, init_values,
                                          params);
  std::cout << "optimize\n";
  gttic_(constraint_manifold);
  auto result = optimizer.optimize();
  gttoc_(constraint_manifold);

  auto problem_dim = optimizer.problemDimension();
  std::cout << "dimension: " << problem_dim.first << " x " << problem_dim.second
            << "\n";
  return result;
}

/** Run optimization using constraint manifold, the manifold tangent space basis
 * and retraction will be performed through manually specified basis variables.
 */
Values optimize_constraint_manifold_specify_variables(
    const EqualityConstraints& constraints, const NonlinearFactorGraph& costs,
    const Values& init_values, const gtsam::BasisKeyFunc& basis_function) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  auto manopt_params = boost::make_shared<ManifoldOptimizer::Params>();
  manopt_params->cc_params->retract_type =
      ConstraintManifold::Params::RetractType::PARTIAL_PROJ;
  manopt_params->cc_params->basis_type =
      ConstraintManifold::Params::BasisType::SPECIFY_VARIABLES;
  std::cout << "building optimizer\n";
  gtsam::ManifoldOptimizerType1 optimizer(costs, constraints, init_values, params,
                                   manopt_params, basis_function);
  std::cout << "optimize\n";
  gttic_(constraint_manifold);
  auto result = optimizer.optimize();
  gttoc_(constraint_manifold);

  auto problem_dim = optimizer.problemDimension();
  std::cout << "dimension: " << problem_dim.first << " x " << problem_dim.second
            << "\n";
  return result;
}

/** Functor version of JointLimitFactor, for creating expressions. Compute error
 * for joint limit error, to reproduce joint limit factor in expressions. */
class JointLimitFunctor {
 protected:
  double low_, high_;

 public:
  JointLimitFunctor(const double& low, const double& high)
      : low_(low), high_(high) {}

  double operator()(const double& q,
                    gtsam::OptionalJacobian<1, 1> H_q = boost::none) const {
    if (q < low_) {
      if (H_q) *H_q = -gtsam::I_1x1;
      return low_ - q;
    } else if (q <= high_) {
      if (H_q) *H_q = gtsam::Z_1x1;
      return 0.0;
    } else {
      if (H_q) *H_q = gtsam::I_1x1;
      return q - high_;
    }
  }
};

}  // namespace gtdynamics
