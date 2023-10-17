/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizerType1.cpp
 * @brief Manifold optimizer implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/ManifoldOptimizerType1.h>
#include <gtdynamics/manifold/SubstituteFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "manifold/Retractor.h"
#include "manifold/TspaceBasis.h"

namespace gtsam {

/* ************************************************************************* */
Values ManifoldOptimizerType1::optimize(
    const NonlinearFactorGraph& costs,
    const gtdynamics::EqualityConstraints& constraints,
    const Values& init_values,
    gtdynamics::ConstrainedOptResult* intermediate_result) const {
  auto mopt_problem = initializeMoptProblem(costs, constraints, init_values);
  return optimize(mopt_problem, intermediate_result);
}

/* ************************************************************************* */
Values ManifoldOptimizerType1::optimize(
    const ManifoldOptProblem& mopt_problem,
    gtdynamics::ConstrainedOptResult* intermediate_result) const {
  auto nonlinear_optimizer = constructNonlinearOptimizer(mopt_problem);
  auto nopt_values = nonlinear_optimizer->optimize();
  if (intermediate_result) {
    intermediate_result->num_iters.push_back(
        std::dynamic_pointer_cast<LevenbergMarquardtOptimizer>(
            nonlinear_optimizer)
            ->getInnerIterations());
  }
  return baseValues(mopt_problem, nopt_values);
}

/* ************************************************************************* */
std::shared_ptr<NonlinearOptimizer>
ManifoldOptimizerType1::constructNonlinearOptimizer(
    const ManifoldOptProblem& mopt_problem) const {
  if (std::holds_alternative<GaussNewtonParams>(nopt_params_)) {
    return std::make_shared<GaussNewtonOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<GaussNewtonParams>(nopt_params_));
  } else if (std::holds_alternative<LevenbergMarquardtParams>(nopt_params_)) {
    return std::make_shared<LevenbergMarquardtOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<LevenbergMarquardtParams>(nopt_params_));
  } else if (std::holds_alternative<DoglegParams>(nopt_params_)) {
    return std::make_shared<DoglegOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<DoglegParams>(nopt_params_));
  } else {
    return std::make_shared<LevenbergMarquardtOptimizer>(mopt_problem.graph_,
                                                         mopt_problem.values_);
  }
}

}  // namespace gtsam
