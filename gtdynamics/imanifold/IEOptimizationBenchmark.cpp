#include "imanifold/IEGDOptimizer.h"
#include "imanifold/IELMOptimizer.h"
#include "optimizer/BarrierOptimizer.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <numeric>

namespace gtsam {

void IEResultSummary::printLatex(std::ostream &latex_os) const {
  latex_os << "& " + exp_name + " & $" << factor_dim << " \\times "
           << variable_dim << "$ & " << total_inner_iters << " & "
           << std::defaultfloat << std::setprecision(2) << cost
           << std::scientific << " & " << std::fixed << std::setprecision(6)
           << e_violation << std::scientific << " & " << std::fixed
           << std::setprecision(6) << i_violation << std::defaultfloat
           << "\\\\\n";
}

/* ************************************************************************* */
IEResultSummary OptimizeSoftConstraints(const IEConsOptProblem &problem,
                                        LevenbergMarquardtParams lm_params,
                                        double mu) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraintsGraph(mu));

  LevenbergMarquardtOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto result = optimizer.optimize();

  IEResultSummary summary;
  summary.exp_name = "soft";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters = optimizer.getInnerIterations();
  summary.total_iters = optimizer.iterations();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  return summary;
}

/* ************************************************************************* */
IEResultSummary
OptimizeBarrierMethod(const IEConsOptProblem &problem,
                      const gtdynamics::BarrierParameters &params) {

  gtdynamics::BarrierOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;
  Values result = optimizer.optimize(
      problem.costs(), problem.eConstraints(), problem.iConstraints(),
      problem.initValues(), &intermediate_result);

  IEResultSummary summary;
  summary.exp_name = "barrier";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters =
      std::accumulate(intermediate_result.num_inner_iters.begin(),
                      intermediate_result.num_inner_iters.end(), 0.0);
  summary.total_iters =
      std::accumulate(intermediate_result.num_iters.begin(),
                      intermediate_result.num_iters.end(), 0.0);
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  return summary;
}

/* ************************************************************************* */
IEResultSummary
OptimizeIEGD(const IEConsOptProblem &problem, const gtsam::GDParams &params,
              const IEConstraintManifold::Params::shared_ptr &iecm_params) {

  IEGDOptimizer optimizer(params);
  Values result = optimizer.optimize(problem.costs(), problem.eConstraints(),
                                     problem.iConstraints(),
                                     problem.initValues(), iecm_params);
  const auto &iter_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "mopt(1st order)";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension() - problem.eConstraints().dim();
  summary.total_inner_iters =
      iter_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iter_details.size();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  return summary;
}

/* ************************************************************************* */
IEResultSummary
OptimizeIELM(const IEConsOptProblem &problem,
              const gtsam::LevenbergMarquardtParams &params,
              const gtsam::IELMParams &ie_params,
              const IEConstraintManifold::Params::shared_ptr &iecm_params) {
  IELMOptimizer optimizer(params, ie_params);
  Values result = optimizer.optimize(problem.costs(), problem.eConstraints(),
                                     problem.iConstraints(),
                                     problem.initValues(), iecm_params);
  const auto &iter_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "mopt(2nd order)";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension() - problem.eConstraints().dim();
  summary.total_inner_iters =
      iter_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iter_details.size();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  return summary;
}

} // namespace gtsam
