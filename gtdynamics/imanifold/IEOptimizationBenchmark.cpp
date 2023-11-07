#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <numeric>

namespace gtsam {

/* ************************************************************************* */
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
void IEResultSummary::exportFile(const std::string &file_path) const {
  std::ofstream file;
  file.open(file_path);
  file << "cost"
       << ","
       << "e_violation"
       << ","
       << "i_violation"
       << ","
       << "num_inner_iters"
       << "\n";
  for (const auto &iter_summary : iters_summary) {
    file << iter_summary.cost << "," << iter_summary.e_violation << ","
         << iter_summary.i_violation << "," << iter_summary.num_inner_iters
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
           << iter_summary.num_inner_iters << "\n";
    }
  }
  file.close();
}

/* ************************************************************************* */
std::pair<IEResultSummary, LMItersDetail>
OptimizeSoftConstraints(const IEConsOptProblem &problem,
                        LevenbergMarquardtParams lm_params, double mu) {
  NonlinearFactorGraph graph = problem.costs_;
  graph.add(problem.constraintsGraph(mu));

  HistoryLMOptimizer optimizer(graph, problem.initValues(), lm_params);
  auto result = optimizer.optimize();

  IEResultSummary summary;
  summary.exp_name = "soft";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.total_inner_iters = optimizer.getInnerIterations();
  summary.total_iters = optimizer.iterations();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);

  const auto &history_states = optimizer.states();
  for (const auto &state : history_states) {
    const Values &iter_values = state.values;
    IEIterSummary iter_summary;
    iter_summary.num_inner_iters = state.totalNumberInnerIterations;
    iter_summary.cost = problem.evaluateCost(iter_values);
    iter_summary.e_violation =
        problem.evaluateEConstraintViolationL2Norm(iter_values);
    iter_summary.i_violation =
        problem.evaluateIConstraintViolationL2Norm(iter_values);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, history_states);
}

/* ************************************************************************* */
std::pair<IEResultSummary, BarrierItersDetail>
OptimizeBarrierMethod(const IEConsOptProblem &problem,
                      const gtdynamics::BarrierParameters &params) {

  gtdynamics::BarrierOptimizer optimizer(params);
  gtdynamics::ConstrainedOptResult intermediate_result;
  Values result = optimizer.optimize(
      problem.costs(), problem.eConstraints(), problem.iConstraints(),
      problem.initValues(), &intermediate_result);

  IEResultSummary summary;
  BarrierItersDetail iters_details;
  summary.exp_name = "barrier";
  summary.variable_dim = result.dim();
  summary.factor_dim = problem.costsDimension() + problem.eConstraints().dim() +
                       problem.iConstraints().dim();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);

  // double total_inner_iters;
  for (int i = 0; i < intermediate_result.mu_values.size(); i++) {
    // total_inner_iters += intermediate_result.num_inner_iters[i];
    const Values &iter_values = intermediate_result.intermediate_values[i];
    IEIterSummary iter_summary;
    iter_summary.num_inner_iters = intermediate_result.num_inner_iters[i];
    iter_summary.cost = problem.evaluateCost(iter_values);
    iter_summary.e_violation =
        problem.evaluateEConstraintViolationL2Norm(iter_values);
    iter_summary.i_violation =
        problem.evaluateIConstraintViolationL2Norm(iter_values);
    iter_summary.mu = intermediate_result.mu_values[i];
    summary.iters_summary.emplace_back(iter_summary);
    iters_details.emplace_back(intermediate_result.mu_values[i],
                               intermediate_result.intermediate_values[i]);
  }
  summary.total_inner_iters = summary.iters_summary.back().num_inner_iters;
  // summary.total_iters = summary.iters_summary.back().num_iters;

  //     std::accumulate(intermediate_result.num_inner_iters.begin(),
  //                     intermediate_result.num_inner_iters.end(), 0.0);
  // summary.total_iters =
  //     std::accumulate(intermediate_result.num_iters.begin(),
  //                     intermediate_result.num_iters.end(), 0.0);

  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, IEGDItersDetails>
OptimizeIEGD(const IEConsOptProblem &problem, const gtsam::GDParams &params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params) {

  IEGDOptimizer optimizer(params);
  Values result = optimizer.optimize(problem.costs(), problem.eConstraints(),
                                     problem.iConstraints(),
                                     problem.initValues(), iecm_params);
  const auto &iters_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "mopt(1st order)";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.size();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    Values state_values = CollectManifoldValues(state.manifolds);
    IEIterSummary iter_summary;
    iter_summary.num_inner_iters = state.totalNumberInnerIterations;
    iter_summary.cost = problem.evaluateCost(state_values);
    iter_summary.e_violation =
        problem.evaluateEConstraintViolationL2Norm(state_values);
    iter_summary.i_violation =
        problem.evaluateIConstraintViolationL2Norm(state_values);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, iters_details);
}

/* ************************************************************************* */
std::pair<IEResultSummary, IELMItersDetails>
OptimizeIELM(const IEConsOptProblem &problem,
             const gtsam::LevenbergMarquardtParams &params,
             const gtsam::IELMParams &ie_params,
             const IEConstraintManifold::Params::shared_ptr &iecm_params) {
  IELMOptimizer optimizer(params, ie_params);
  Values result = optimizer.optimize(problem.costs(), problem.eConstraints(),
                                     problem.iConstraints(),
                                     problem.initValues(), iecm_params);
  const auto &iters_details = optimizer.details();

  IEResultSummary summary;
  summary.exp_name = "mopt(2nd order)";
  summary.variable_dim = result.dim() - problem.eConstraints().dim();
  summary.factor_dim = problem.costsDimension();
  summary.total_inner_iters =
      iters_details.back().state.totalNumberInnerIterations;
  summary.total_iters = iters_details.size();
  summary.cost = problem.evaluateCost(result);
  summary.e_violation = problem.evaluateEConstraintViolationL2Norm(result);
  summary.i_violation = problem.evaluateIConstraintViolationL2Norm(result);
  for (const auto &iter_detail : iters_details) {
    const auto &state = iter_detail.state;
    Values state_values = state.baseValues();
    IEIterSummary iter_summary;
    iter_summary.num_inner_iters = state.totalNumberInnerIterations;
    iter_summary.cost = problem.evaluateCost(state_values);
    iter_summary.e_violation =
        problem.evaluateEConstraintViolationL2Norm(state_values);
    iter_summary.i_violation =
        problem.evaluateIConstraintViolationL2Norm(state_values);
    summary.iters_summary.emplace_back(iter_summary);
  }
  return std::make_pair(summary, iters_details);
}

} // namespace gtsam
