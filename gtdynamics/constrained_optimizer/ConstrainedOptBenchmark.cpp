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

#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/cmopt/TspaceBasis.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>

#include <iomanip>

namespace gtdynamics {

void PrintLatex(std::ostream &latex_os, std::string exp_name, size_t f_dim,
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
Values OptimizeE_SoftConstraints(const EConsOptProblem &problem,
                                 std::ostream &latex_os,
                                 LevenbergMarquardtParams lm_params, double mu,
                                 double constraint_unit_scale) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraints().penaltyGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(latex_os, "Soft Constraint",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             optimizer.getInnerIterations(),
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));
  return result;
}

/* ************************************************************************* */
ManifoldOptimizerParameters DefaultMoptParams() {
  ManifoldOptimizerParameters mopt_params;
  auto retractor_params = std::make_shared<RetractParams>();
  mopt_params.cc_params->retractor_creator =
      std::make_shared<UoptRetractorCreator>(retractor_params);
  auto basis_params = std::make_shared<TspaceBasisParams>();
  basis_params->always_construct_basis = false;
  mopt_params.cc_params->basis_creator =
      std::make_shared<OrthonormalBasisCreator>(basis_params);
  return mopt_params;
}

/* ************************************************************************* */
ManifoldOptimizerParameters
DefaultMoptParamsSV(const BasisKeyFunc &basis_key_func) {
  ManifoldOptimizerParameters mopt_params;
  auto retractor_params = std::make_shared<RetractParams>();
  retractor_params->use_basis_keys = true;
  mopt_params.cc_params->retractor_creator =
      std::make_shared<BasisRetractorCreator>(basis_key_func, retractor_params);
  auto basis_params = std::make_shared<TspaceBasisParams>();
  basis_params->use_basis_keys = true;
  basis_params->always_construct_basis = false;
  mopt_params.cc_params->basis_creator =
      std::make_shared<EliminationBasisCreator>(basis_params);
  return mopt_params;
}

/* ************************************************************************* */
Values OptimizeE_CMOpt(const EConsOptProblem &problem, std::ostream &latex_os,
                       ManifoldOptimizerParameters mopt_params,
                       LevenbergMarquardtParams lm_params, std::string exp_name,
                       double constraint_unit_scale) {
  NonlinearMOptimizer optimizer(mopt_params, lm_params);
  auto mopt_problem = optimizer.initializeMoptProblem(
      problem.costs(), problem.constraints(), problem.initValues());

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize(mopt_problem);
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  auto problem_dim = mopt_problem.problemDimension();
  PrintLatex(latex_os, "\\textbf{" + exp_name + "}", problem_dim.first,
             problem_dim.second, optimization_time,
             //   intermediate_result.num_iters.at(0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values OptimizeE_Penalty(const EConsOptProblem &problem, std::ostream &latex_os,
                         gtsam::PenaltyOptimizerParams::shared_ptr params,
                         double constraint_unit_scale) {
  // params->store_iter_details = true; // Not yet in gtsam, TODO
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::PenaltyOptimizer optimizer(graph, problem.initValues(), params);

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::microseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-6;

  PrintLatex(latex_os, "Penalty Method",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             //   std::accumulate(intermediate_result.num_iters.begin(),
             //                   intermediate_result.num_iters.end(), 0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

/* ************************************************************************* */
Values
OptimizeE_AugmentedLagrangian(const EConsOptProblem &problem,
                              std::ostream &latex_os,
                              gtsam::AugmentedLagrangianParams::shared_ptr params,
                              double constraint_unit_scale) {
  gtsam::NonlinearFactorGraph graph = problem.costs();
  graph.add(problem.constraints());
  gtsam::AugmentedLagrangianOptimizer optimizer(graph, problem.initValues(),
                                                params);

  auto optimization_start = std::chrono::system_clock::now();
  auto result = optimizer.optimize();
  auto optimization_end = std::chrono::system_clock::now();
  auto optimization_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                            optimization_start);
  double optimization_time = optimization_time_ms.count() * 1e-3;

  PrintLatex(latex_os, "Augmented Lagrangian",
             problem.costsDimension() + problem.constraintsDimension(),
             problem.valuesDimension(), optimization_time,
             //   std::accumulate(intermediate_result.num_iters.begin(),
             //                   intermediate_result.num_iters.end(), 0), TODO
             0,
             problem.evaluateEConstraintViolationL2Norm(result) *
                 constraint_unit_scale,
             problem.evaluateCost(result));

  return result;
}

} // namespace gtdynamics
