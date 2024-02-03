#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

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
  std::cout << "evaluate iter " << iterations << "\n";
  cost = problem.evaluateCost(values);
  e_violation = problem.evaluateEConstraintViolationL2Norm(values);
  i_violation = problem.evaluateIConstraintViolationL2Norm(values);
  if (iterations > 60) {
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
  latex_os << "& " + exp_name + " & $" << factor_dim << " \\times "
           << variable_dim << "$ & " << total_iters << " & ";
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
    }
    else if (error > 1e3) {
      latex_os << std::scientific << std::setprecision(1) << error;
    } else {
      latex_os << std::fixed << std::setprecision(2) << error;
    }
    latex_os << " & ";
    if (error_proj < 1e-8) {
      latex_os << 0;
    }
    else if (error_proj > 1e3) {
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
  file << "iterations,"
       << "num_inner_iters,"
       << "cost,"
       << "e_violation,"
       << "i_violation,"
       << "proj_cost"
       << "\n";
  for (const auto &iter_summary : iters_summary) {
    file << iter_summary.iterations << "," << iter_summary.inner_iters << ","
         << iter_summary.cost << "," << iter_summary.e_violation << ","
         << iter_summary.i_violation << "," << iter_summary.projected_cost
         << "\n";
  }
  file.close();
}

/* ************************************************************************* */
void IEResultSummary::exportFileWithMu(const std::string &file_path) const {
  std::ofstream file;
  file.open(file_path);
  file << "mu"
       << ","
       << "cost"
       << ","
       << "e_violation"
       << ","
       << "i_violation"
       << ","
       << "num_inner_iters"
       << "\n";
  for (int i = 0; i < iters_summary.size(); i++) {
    if (i == iters_summary.size() - 1 ||
        (i > 0 && iters_summary[i].mu != iters_summary[i + 1].mu)) {
      const auto &iter_summary = iters_summary.at(i);
      file << iter_summary.mu << "," << iter_summary.cost << ","
           << iter_summary.e_violation << "," << iter_summary.i_violation << ","
           << iter_summary.inner_iters << "\n";
    }
  }
  file.close();
}

/* ************************************************************************* */
std::pair<IEResultSummary, LMItersDetail>
OptimizeSoftConstraints(const IEConsOptProblem &problem,
                        LevenbergMarquardtParams lm_params, double mu,
                        bool eval_projected_cost) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraintsGraph(mu));

  HistoryLMOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto result = optimizer.optimize();

  IEResultSummary summary;
  summary.exp_name = "Soft";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.total_inner_iters = optimizer.getInnerIterations();
  summary.total_iters = optimizer.iterations();
  summary.values = result;
  summary.evaluate(problem);

  const auto &history_states = optimizer.states();
  for (const auto &state : history_states) {
    const Values &iter_values = state.values;
    IEIterSummary iter_summary;
    iter_summary.iterations = state.iterations;
    iter_summary.inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, iter_values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, history_states);
}

/* ************************************************************************* */
std::pair<IEResultSummary, BarrierItersDetail>
OptimizePenaltyMethod(const IEConsOptProblem &problem,
                      const gtdynamics::BarrierParameters::shared_ptr &params,
                      bool eval_projected_cost) {

  gtdynamics::BarrierOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;
  Values result = optimizer.optimize(
      problem.costs(), problem.eConstraints(), problem.iConstraints(),
      problem.initValues(), &intermediate_result);
  std::cout << "optimize finished\n";

  IEResultSummary summary;
  BarrierItersDetail iters_details;
  summary.exp_name = "Penalty";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);
  summary.total_iters = intermediate_result.num_iters.back();
  // double total_inner_iters;
  for (int i = 0; i < intermediate_result.mu_values.size(); i++) {
    // total_inner_iters += intermediate_result.num_inner_iters[i];
    const Values &iter_values = intermediate_result.intermediate_values[i];
    IEIterSummary iter_summary;
    iter_summary.iterations = intermediate_result.num_iters[i];
    iter_summary.inner_iters = intermediate_result.num_inner_iters[i];
    iter_summary.evaluate(problem, iter_values, eval_projected_cost);
    iter_summary.mu = intermediate_result.mu_values[i];
    summary.iters_summary.emplace_back(iter_summary);
    iters_details.emplace_back(intermediate_result.mu_values[i],
                               intermediate_result.intermediate_values[i]);
  }
  summary.total_inner_iters = summary.iters_summary.back().inner_iters;
  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, SQPItersDetails>
OptimizeSQP(const IEConsOptProblem &problem,
            const SQPParams::shared_ptr &params, bool eval_projected_cost) {
  SQPOptimizer optimizer(*params);
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  auto iters_details = optimizer.details();
  IEResultSummary summary;
  summary.exp_name = "SQP";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.values = result;
  summary.evaluate(problem);
  summary.total_iters = iters_details.back().state.iterations;
  for (const auto &iter_details : optimizer.details()) {
    const auto &state = iter_details.state;
    IEIterSummary iter_summary;
    iter_summary.iterations = state.iterations;
    iter_summary.inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return {summary, iters_details};
}

/* ************************************************************************* */
std::pair<IEResultSummary, IEGDItersDetails>
OptimizeIEGD(const IEConsOptProblem &problem, const gtsam::GDParams &params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params,
             bool eval_projected_cost) {

  IEGDOptimizer optimizer(params, iecm_params);
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  const auto &iters_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "CMOpt(IE-GD)";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  summary.values = result;
  summary.evaluate(problem);
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    Values state_values = state.manifolds.baseValues();
    IEIterSummary iter_summary;
    iter_summary.iterations = state.iterations;
    iter_summary.inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state_values, eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeELM(const IEConsOptProblem &problem,
            const gtsam::IELMParams &ielm_params,
            const IEConstraintManifold::Params::shared_ptr &iecm_params,
            double mu, bool eval_projected_cost) {
  IELMOptimizer optimizer(ielm_params, iecm_params);
  NonlinearFactorGraph merit_graph = problem.costs();
  merit_graph.add(problem.iConstraints().meritGraph(mu));
  gtdynamics::InequalityConstraints i_constraints;
  Values result = optimizer.optimize(merit_graph, problem.eConstraints(),
                                     i_constraints, problem.initValues());
  const auto &iters_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "CM-Opt";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension() + problem.iConstraints().dim();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  summary.values = result;
  summary.evaluate(problem);
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    IEIterSummary iter_summary;
    iter_summary.iterations = state.iterations;
    iter_summary.inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.baseValues(), eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeIELM(const IEConsOptProblem &problem,
             const gtsam::IELMParams &ielm_params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params,
             std::string exp_name, bool eval_projected_cost) {
  IELMOptimizer optimizer(ielm_params, iecm_params);
  Values result =
      optimizer.optimize(problem.costs(), problem.eConstraints(),
                         problem.iConstraints(), problem.initValues());
  const auto &iters_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = exp_name;
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.back().state.iterations;
  summary.values = result;
  summary.evaluate(problem);
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    IEIterSummary iter_summary;
    iter_summary.iterations = state.iterations;
    iter_summary.inner_iters = state.totalNumberInnerIterations;
    iter_summary.evaluate(problem, state.baseValues(), eval_projected_cost);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, iters_details);
}

} // namespace gtsam
