#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmarkIE.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtdynamics/utils/Timer.h>

namespace gtsam {

/* ************************************************************************* */
void IEItersSummary::addAccumulative(const IEConsOptProblem &problem,
                                     const Values &values,
                                     const size_t step_lm_iters,
                                     const size_t step_lm_inner_iters,
                                     bool eval_projected_cost) {
  size_t accum_iters = step_lm_iters;
  size_t accum_inner_iters = step_lm_inner_iters;
  if (size() > 0) {
    accum_iters += back().accum_iters;
    accum_inner_iters += back().accum_inner_iters;
  }
  emplace_back(problem, values, accum_iters, accum_inner_iters,
               eval_projected_cost);
}

/* ************************************************************************* */
Values ProjectValues(const IEConsOptProblem &problem, const Values &values,
                     double sigma) {
  NonlinearFactorGraph graph = problem.eConstraints().meritGraph();
  graph.add(problem.iConstraints().meritGraph());

  LevenbergMarquardtParams lm_params;
  lm_params.setlambdaUpperBound(1e10);
  lm_params.setErrorTol(1e-30);
  lm_params.setAbsoluteErrorTol(1e-30);
  lm_params.setRelativeErrorTol(1e-30);
  // lm_params.setVerbosityLM("SUMMARY");

  // First stage, optimize with priors
  NonlinearFactorGraph graph_wp = graph;
  AddGeneralPriors(values, sigma, graph_wp);
  LevenbergMarquardtOptimizer optimizer_wp(graph, values, lm_params);
  auto result_wp = optimizer_wp.optimize();

  // Second stage, optimize without priors
  LevenbergMarquardtOptimizer optimizer(graph, result_wp, lm_params);
  Values projected_values = optimizer.optimize();
  return projected_values;
}

/* ************************************************************************* */
void IEIterSummary::evaluate(const IEConsOptProblem &problem,
                             const Values &values, bool eval_projected_cost) {
  std::cout << "evaluate iter " << accum_iters << "\n";
  cost = problem.evaluateCost(values);
  e_violation = problem.evaluateEConstraintViolationL2Norm(values);
  i_violation = problem.evaluateIConstraintViolationL2Norm(values);
  if (accum_iters > 60) {
    return;
  }
  if (eval_projected_cost) {
    Values projected_values = ProjectValues(problem, values);
    projected_cost = problem.evaluateCost(projected_values);
  }
}

/* ************************************************************************* */
void IEResultSummary::evaluate(const IEConsOptProblem &problem) {
  std::cout << "evaluate summary " << total_iters << "\n";
  cost = problem.evaluateCost(values);
  e_violation = problem.evaluateEConstraintViolationL2Norm(values);
  i_violation = problem.evaluateIConstraintViolationL2Norm(values);
  projected_values = ProjectValues(problem, values);
  projected_cost = problem.evaluateCost(projected_values);
}

/* ************************************************************************* */
void IEResultSummary::printLatex(std::ostream &latex_os) const {
  latex_os << "& " + exp_name + " & ";
  latex_os << "$" << factor_dim << " \\times " << variable_dim << "$ & ";
  latex_os << std::scientific << std::setprecision(1) << optimization_time
           << " & ";
  latex_os << total_iters << " & ";
  if (e_violation < 1e-8) {
    latex_os << 0 << " & ";
  } else {
    latex_os << std::scientific << std::setprecision(1) << e_violation << " & ";
  }

  if (i_violation < 1e-8) {
    latex_os << 0 << " & ";
  } else {
    latex_os << std::scientific << std::setprecision(1) << i_violation << " & ";
  }

  if (cost > 1e3) {
    latex_os << std::scientific << std::setprecision(1) << cost << " & ";
  } else {
    latex_os << std::fixed << std::setprecision(2) << cost << " & ";
  }

  if (projected_cost > 1e3) {
    latex_os << std::scientific << std::setprecision(1) << projected_cost
             << "\\\\\n";
  } else {
    latex_os << std::fixed << std::setprecision(2) << projected_cost
             << "\\\\\n";
  }
}

/* ************************************************************************* */
void EvaluateCostTerms(std::ostream &latex_os,
                       const std::vector<NonlinearFactorGraph> &graphs,
                       const Values &values, const Values &proj_values,
                       std::string exp_name) {
  latex_os << "& " + exp_name;
  for (const auto &graph : graphs) {
    double error = graph.error(values);
    double error_proj = graph.error(proj_values);
    latex_os << " & ";
    if (error < 1e-8) {
      latex_os << 0;
    } else if (error > 1e3) {
      latex_os << std::scientific << std::setprecision(1) << error;
    } else {
      latex_os << std::fixed << std::setprecision(2) << error;
    }
    latex_os << " & ";
    if (error_proj < 1e-8) {
      latex_os << 0;
    } else if (error_proj > 1e3) {
      latex_os << std::scientific << std::setprecision(1) << error_proj;
    } else {
      latex_os << std::fixed << std::setprecision(2) << error_proj;
    }
  }
  latex_os << "\\\\\n";
}

/* ************************************************************************* */
void IEResultSummary::exportFile(const std::string &file_path) const {
  std::ofstream file;
  file.open(file_path);
  file << "accum_iters,"
       << "accum_inner_iters,"
       << "cost,"
       << "e_violation,"
       << "i_violation,"
       << "proj_cost"
       << "\n";
  for (const auto &iter_summary : iters_summary) {
    file << iter_summary.accum_iters << "," << iter_summary.accum_inner_iters
         << "," << iter_summary.cost << "," << iter_summary.e_violation << ","
         << iter_summary.i_violation << "," << iter_summary.projected_cost
         << "\n";
  }
  file.close();
}

/* ************************************************************************* */
std::pair<IEResultSummary, LMItersDetail>
OptimizeIE_Soft(const IEConsOptProblem &problem,
                LevenbergMarquardtParams lm_params, double mu,
                bool eval_projected_cost) {
  /// Run optimization.
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraints().meritGraph(mu));
  graph.add(problem.iConstraints().meritGraph(mu));
  HistoryLMOptimizer optimizer(graph, problem.initValues(), lm_params);

  Timer timer;
  timer.start();
  auto result = optimizer.optimize();
  timer.stop();

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "Soft";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.total_inner_iters = optimizer.getInnerIterations();
  summary.total_iters = optimizer.iterations();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &history_states = optimizer.states();
  for (const auto &state : history_states) {
    const Values &iter_values = state.values;
    IEIterSummary iter_summary;
    iter_summary.accum_iters = state.iterations;
    iter_summary.accum_inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, iter_values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, history_states);
}

/* ************************************************************************* */
std::pair<IEResultSummary, PenaltyItersDetails>
OptimizeIE_Penalty(const IEConsOptProblem &problem,
                   const gtsam::PenaltyParameters::shared_ptr &params,
                   bool eval_projected_cost) {
  /// Run optimization.
  params->store_iter_details = true;
  gtsam::PenaltyOptimizer optimizer(params);

  Timer timer;
  timer.start();
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  timer.stop();
  std::cout << "optimize finished\n";

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "Penalty";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &iters_details = optimizer.details();
  for (const auto &iter_detail : iters_details) {
    if (params->store_lm_details) {
      for (int i = 0; i < iter_detail.lm_iters_values.size(); i++) {
        const auto &iter_values = iter_detail.lm_iters_values.at(i);
        size_t inner_lm_iters = iter_detail.lm_inner_iters.at(i);
        summary.iters_summary.addAccumulative(
            problem, iter_values, 1, inner_lm_iters, eval_projected_cost);
      }
    } else {
      summary.iters_summary.addAccumulative(
          problem, iter_detail.values, iter_detail.num_lm_iters,
          iter_detail.num_lm_inner_iters, eval_projected_cost);
    }
  }
  summary.total_iters = summary.iters_summary.back().accum_iters;
  summary.total_inner_iters = summary.iters_summary.back().accum_inner_iters;
  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, AugmentedLagrangianItersDetails>
OptimizeIE_AugmentedLagrangian(
    const IEConsOptProblem &problem,
    const gtsam::AugmentedLagrangianParameters::shared_ptr &params,
    bool eval_projected_cost) {
  /// Run optimization.
  params->store_iter_details = true;
  gtsam::AugmentedLagrangianOptimizer optimizer(params);
  Timer timer;
  timer.start();
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  timer.stop();
  std::cout << "optimize finished\n";

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "AugL";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &iters_details = optimizer.details();
  for (const auto &iter_detail : iters_details) {
    if (params->store_lm_details) {
      for (int i = 0; i < iter_detail.lm_iters_values.size(); i++) {
        const auto &iter_values = iter_detail.lm_iters_values.at(i);
        size_t inner_lm_iters = iter_detail.lm_inner_iters.at(i);
        summary.iters_summary.addAccumulative(
            problem, iter_values, 1, inner_lm_iters, eval_projected_cost);
      }
    } else {
      summary.iters_summary.addAccumulative(
          problem, iter_detail.values, iter_detail.num_lm_iters,
          iter_detail.num_lm_inner_iters, eval_projected_cost);
    }
  }
  summary.total_iters = summary.iters_summary.back().accum_iters;
  summary.total_inner_iters = summary.iters_summary.back().accum_inner_iters;
  return {summary, iters_details};
}

/* ************************************************************************* */
std::pair<IEResultSummary, SQPItersDetails>
OptimizeIE_SQP(const IEConsOptProblem &problem,
               const SQPParams::shared_ptr &params, bool eval_projected_cost) {
  /// Run optimization.
  SQPOptimizer optimizer(params);
  Timer timer;
  timer.start();
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  timer.stop();

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "SQP";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  auto iters_details = optimizer.details();
  summary.total_iters = iters_details.back().state.iterations;
  for (const auto &iter_details : optimizer.details()) {
    const auto &state = iter_details.state;
    IEIterSummary iter_summary;
    iter_summary.accum_iters = state.iterations;
    iter_summary.accum_inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return {summary, iters_details};
}

/* ************************************************************************* */
std::pair<IEResultSummary, IEGDItersDetails>
OptimizeIE_CMCOptGD(const IEConsOptProblem &problem,
                    const gtsam::GDParams &params,
                    const IEConstraintManifold::Params::shared_ptr &iecm_params,
                    bool eval_projected_cost) {
  /// Run optimization.
  IEGDOptimizer optimizer(params, iecm_params);
  Timer timer;
  timer.start();
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  timer.stop();

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "CMOpt(IE-GD)";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();

  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &iters_details = optimizer.details();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    Values state_values = state.manifolds.baseValues();
    IEIterSummary iter_summary;
    iter_summary.accum_iters = state.iterations;
    iter_summary.accum_inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state_values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return {summary, iters_details};
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeIE_CMOpt(const IEConsOptProblem &problem,
                 const gtsam::IELMParams &ielm_params,
                 const IEConstraintManifold::Params::shared_ptr &iecm_params,
                 double mu, bool eval_projected_cost) {
  /// Run optimization.
  IELMOptimizer optimizer(ielm_params, iecm_params);
  NonlinearFactorGraph merit_graph = problem.costs();
  merit_graph.add(problem.iConstraints().meritGraph(mu));
  gtsam::InequalityConstraints i_constraints;
  Timer timer;
  timer.start();
  Values result = optimizer.optimize(merit_graph, problem.eConstraints(),
                                     i_constraints, problem.initValues());
  timer.stop();

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = "CM-Opt";
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension() + problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &iters_details = optimizer.details();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    IEIterSummary iter_summary;
    iter_summary.accum_iters = state.iterations;
    iter_summary.accum_inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.baseValues(), eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return {summary, iters_details};
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeIE_CMCOptLM(const IEConsOptProblem &problem,
                    const gtsam::IELMParams &ielm_params,
                    const IEConstraintManifold::Params::shared_ptr &iecm_params,
                    std::string exp_name, bool eval_projected_cost) {
  /// Run optimization.
  IELMOptimizer optimizer(ielm_params, iecm_params);
  Timer timer;
  timer.start();
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  timer.stop();

  /// Summary of optimization result.
  IEResultSummary summary;
  summary.exp_name = exp_name;
  summary.optimization_time = timer.seconds();
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();
  summary.values = result;
  summary.evaluate(problem);

  /// Summary of each iteration.
  const auto &iters_details = optimizer.details();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    IEIterSummary iter_summary;
    iter_summary.accum_iters = state.iterations;
    iter_summary.accum_inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.baseValues(), eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return {summary, iters_details};
}

} // namespace gtsam
