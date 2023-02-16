/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  OptimizationBenchmark.cpp
 * @brief Constrained optimization benchmark implementations.
 * @author Yetong Zhang
 */

#include <gtdynamics/optimizer/OptimizationBenchmark.h>

using gtsam::LevenbergMarquardtParams, gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph, gtsam::Values;

namespace gtdynamics {

void PrintLatex(std::ostream& latex_os, std::string exp_name, size_t f_dim,
                size_t v_dim, double time, size_t num_iters,
                double constraint_vio, double cost) {
  latex_os << "& " + exp_name + " & $" << f_dim << " \\times " << v_dim
           << "$ & " << std::setprecision(4) << time << std::defaultfloat
           << " & " << num_iters << " & " << std::scientific
           << std::setprecision(2) << constraint_vio << std::defaultfloat
           << " & " << std::fixed << std::setprecision(2) << cost
           << std::defaultfloat << "\\\\\n";
}

/* ************************************************************************* */
Values OptimizeSoftConstraints(const EqConsOptProblem& problem,
                               std::ostream& latex_os,
                               LevenbergMarquardtParams lm_params, double mu,
                               double constraint_unit_scale) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraintsGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(
      latex_os, "Soft Constraint",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimization_time,
      optimizer.getInnerIterations(),
      problem.evaluateConstraintViolationL2Norm(result) * constraint_unit_scale,
      problem.evaluateCost(result));
  return result;
}

/* ************************************************************************* */
gtsam::ManifoldOptimizerParameters DefaultMoptParams() {
  gtsam::ManifoldOptimizerParameters mopt_params;
  mopt_params.cc_params->basis_params->always_construct_basis = false;
  return mopt_params;
}

/* ************************************************************************* */
gtsam::ManifoldOptimizerParameters DefaultMoptParamsSV() {
  gtsam::ManifoldOptimizerParameters mopt_params = DefaultMoptParams();
  mopt_params.cc_params->retract_params->setFixVars();
  mopt_params.cc_params->basis_params->setFixVars();
  return mopt_params;
}

/* ************************************************************************* */
Values OptimizeConstraintManifold(
    const EqConsOptProblem& problem, std::ostream& latex_os,
    gtsam::ManifoldOptimizerParameters mopt_params,
    LevenbergMarquardtParams lm_params, std::string exp_name,
    double constraint_unit_scale) {
  gtsam::ManifoldOptimizerType1 optimizer(mopt_params, lm_params);
  auto mopt_problem = optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());
  gtdynamics::ConstrainedOptResult intermediate_result;

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize(mopt_problem, &intermediate_result);
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  auto problem_dim = mopt_problem.problemDimension();
  PrintLatex(
      latex_os, "\\textbf{" + exp_name + "}", problem_dim.first,
      problem_dim.second, optimization_time,
      intermediate_result.num_iters.at(0),
      problem.evaluateConstraintViolationL2Norm(result) * constraint_unit_scale,
      problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values OptimizePenaltyMethod(const EqConsOptProblem& problem,
                             std::ostream& latex_os,
                             PenaltyMethodParameters params,
                             double constraint_unit_scale) {
  PenaltyMethodOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize(problem.costs(), problem.constraints(),
                                   problem.initValues(), &intermediate_result);
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(
      latex_os, "Penalty Method",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimization_time,
      std::accumulate(intermediate_result.num_iters.begin(),
                      intermediate_result.num_iters.end(), 0),
      problem.evaluateConstraintViolationL2Norm(result) * constraint_unit_scale,
      problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values OptimizeAugmentedLagrangian(const EqConsOptProblem& problem,
                                   std::ostream& latex_os,
                                   AugmentedLagrangianParameters params,
                                   double constraint_unit_scale) {
  AugmentedLagrangianOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize(problem.costs(), problem.constraints(),
                                   problem.initValues(), &intermediate_result);
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(
      latex_os, "Augmented Lagrangian",
      problem.costsDimension() + problem.constraintsDimension(),
      problem.valuesDimension(), optimization_time,
      std::accumulate(intermediate_result.num_iters.begin(),
                      intermediate_result.num_iters.end(), 0),
      problem.evaluateConstraintViolationL2Norm(result) * constraint_unit_scale,
      problem.evaluateCost(result));

  return result;
}

}  // namespace gtdynamics
