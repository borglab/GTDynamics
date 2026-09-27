/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NonlinearManifoldOptimizer.cpp
 * @brief Manifold optimizer implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/cmopt/NonlinearManifoldOptimizer.h>
#include <gtdynamics/factors/SubstituteFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

/* ************************************************************************* */
Values NonlinearManifoldOptimizer::optimize(const NonlinearFactorGraph& costs,
                                     const NonlinearEqualityConstraints& constraints,
                                     const Values& init_values) const {
  auto mopt_problem = initializeManifoldOptimizationProblem(costs, constraints, init_values);
  return optimizeManifoldProblem(mopt_problem);
}

/* ************************************************************************* */
Values NonlinearManifoldOptimizer::optimizeManifoldProblem(
    const ManifoldOptimizationProblem& mopt_problem) const {
  auto nonlinear_optimizer = constructNonlinearOptimizer(mopt_problem);
  auto nopt_values = nonlinear_optimizer->optimize();
  // if (intermediate_result) {
  //   intermediate_result->num_iters.push_back(
  //       std::dynamic_pointer_cast<LevenbergMarquardtOptimizer>(
  //           nonlinear_optimizer)
  //           ->getInnerIterations());
  // }
  return baseValues(mopt_problem, nopt_values);
}

/* ************************************************************************* */
std::shared_ptr<NonlinearOptimizer>
NonlinearManifoldOptimizer::constructNonlinearOptimizer(
    const ManifoldOptimizationProblem& mopt_problem) const {
  if (std::holds_alternative<GaussNewtonParams>(nonlinearOptimizerParams_)) {
    return std::make_shared<gtsam::GaussNewtonOptimizer>(
        mopt_problem.graph, mopt_problem.values,
        std::get<GaussNewtonParams>(nonlinearOptimizerParams_));
  } else if (std::holds_alternative<LevenbergMarquardtParams>(nonlinearOptimizerParams_)) {
    return std::make_shared<gtsam::LevenbergMarquardtOptimizer>(
        mopt_problem.graph, mopt_problem.values,
        std::get<LevenbergMarquardtParams>(nonlinearOptimizerParams_));
  } else if (std::holds_alternative<DoglegParams>(nonlinearOptimizerParams_)) {
    return std::make_shared<gtsam::DoglegOptimizer>(
        mopt_problem.graph, mopt_problem.values,
        std::get<DoglegParams>(nonlinearOptimizerParams_));
  } else {
    return std::make_shared<gtsam::LevenbergMarquardtOptimizer>(
        mopt_problem.graph, mopt_problem.values);
  }
}

}  // namespace gtdynamics
