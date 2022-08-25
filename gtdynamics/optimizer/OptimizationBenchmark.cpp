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

/* ************************************************************************* */
Values OptimizeSoftConstraints(const EqConsOptProblem& problem,
                                 std::ostream& latex_os,
                                 LevenbergMarquardtParams lm_params,
                                 double mu) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraintsGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lm_params);
  boost::timer::cpu_timer optimization_timer;
  optimization_timer.start();
  auto result = optimizer.optimize();
  double optimization_time = 1e-9 * optimization_timer.elapsed().wall;

  latex_os << "& Soft Constraint & $" << problem.costsDimension() + problem.constraintsDimension() << " \\times "
           << problem.valuesDimension() << "$ & " << optimization_time << " & " << optimizer.getInnerIterations() << " & "
           << problem.evaluateConstraintViolationL2Norm(result) << " & " << problem.evaluateCost(result) << "\\\\\n";
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
Values OptimizeConstraintManifold(const EqConsOptProblem& problem,
                                 std::ostream& latex_os,
                                 gtsam::ManifoldOptimizerParameters mopt_params,
                                 LevenbergMarquardtParams lm_params,
                                 std::string exp_name) {
  gtsam::ManifoldOptimizerType1 optimizer(mopt_params, lm_params);
  auto mopt_problem =
      optimizer.initializeMoptProblem(problem.costs(), problem.constraints(), problem.initValues());
  gtdynamics::ConstrainedOptResult intermediate_result;

  boost::timer::cpu_timer optimization_timer;
  optimization_timer.start();
  auto result = optimizer.optimize(mopt_problem, &intermediate_result);
  double optimization_time = 1e-9 * optimization_timer.elapsed().wall;

  auto problem_dim = mopt_problem.problemDimension();
  latex_os << "& \\textbf{" << exp_name << "} & $" << problem_dim.first << " \\times "
           << problem_dim.second << "$ & " << optimization_time << " & " << intermediate_result.num_iters.at(0) << " & "
           << problem.evaluateConstraintViolationL2Norm(result) << " & " << problem.evaluateCost(result) << "\\\\\n";
  return result;
}

/* ************************************************************************* */
Values OptimizePenaltyMethod(const EqConsOptProblem& problem,
                                 std::ostream& latex_os,
                                 PenaltyMethodParameters params) {
  PenaltyMethodOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;

  boost::timer::cpu_timer optimization_timer;
  optimization_timer.start();
  auto result = optimizer.optimize(problem.costs(), problem.constraints(), problem.initValues(), &intermediate_result);
  double optimization_time = 1e-9 * optimization_timer.elapsed().wall;

  latex_os << "& Penalty Method & $"
           << problem.costsDimension() + problem.constraintsDimension()
           << " \\times " << problem.valuesDimension() << "$ & "
           << optimization_time << " & "
           << std::accumulate(intermediate_result.num_iters.begin(),
                              intermediate_result.num_iters.end(), 0)
           << " & " << problem.evaluateConstraintViolationL2Norm(result)
           << " & " << problem.evaluateCost(result) << "\\\\\n";

  return result;
}

/* ************************************************************************* */
Values OptimizeAugmentedLagrangian(const EqConsOptProblem& problem,
                                 std::ostream& latex_os,
                                 AugmentedLagrangianParameters params) {
  AugmentedLagrangianOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;

  boost::timer::cpu_timer optimization_timer;
  optimization_timer.start();
  auto result = optimizer.optimize(problem.costs(), problem.constraints(), problem.initValues(), &intermediate_result);
  double optimization_time = 1e-9 * optimization_timer.elapsed().wall;

  latex_os << "& Augmented Lagrangian & $"
           << problem.costsDimension() + problem.constraintsDimension()
           << " \\times " << problem.valuesDimension() << "$ & "
           << optimization_time << " & "
           << std::accumulate(intermediate_result.num_iters.begin(),
                              intermediate_result.num_iters.end(), 0)
           << " & " << problem.evaluateConstraintViolationL2Norm(result)
           << " & " << problem.evaluateCost(result) << "\\\\\n";

  return result;
}

} // namespace gtdynamics
