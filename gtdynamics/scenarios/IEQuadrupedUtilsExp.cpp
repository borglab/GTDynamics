#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

namespace gtdynamics {
using namespace gtsam;
/* ************************************************************************* */
void EvaluateAndExportIELMResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, IELMItersDetails> &ielm_result,
    const std::string &scenario_folder, bool print_values,
    bool print_iter_details) {
  size_t num_steps = vision60_multi_phase.numSteps();

  //// Visualize optimization progress
  Values result_values = ielm_result.second.back().state.baseValues();
  if (print_iter_details) {
    for (const auto &iter_details : ielm_result.second) {
      IEOptimizer::PrintIterDetails(
          iter_details, num_steps, false, IEVision60Robot::PrintValues,
          IEVision60Robot::PrintDelta, GTDKeyFormatter);
    }
  }

  //// Visualize optimized trajectory.
  if (print_values) {
    IEVision60Robot::PrintValues(result_values, num_steps);
  }

  /// Show active constraints
  std::cout << "active constriants:\n";
  for (const auto &[key, manifold] :
       ielm_result.second.back().state.manifolds) {
    for (const auto &i_idx : manifold.activeIndices()) {
      auto constraint = manifold.iConstraints()->at(i_idx);
      constraint->print();
    }
  }

  //// Evaluate
  problem.eval_func(result_values);
  vision60_multi_phase.evaluateCollocation(result_values);
  for (const auto &iter_details : ielm_result.second) {
    Values values = iter_details.state.baseValues();
    std::cout << iter_details.state.iterations << "\t";
    for (size_t phase_idx = 0;
         phase_idx < vision60_multi_phase.phase_num_steps_.size();
         phase_idx++) {
      double dt = values.atDouble(PhaseKey(phase_idx));
      std::cout << "\t" << dt;
    }
    std::cout << "\n";
  }

  //// Export final optimized trajectory.
  ExportValuesToFile(result_values, scenario_folder + "manopt_values.dat");
  IEVision60Robot::ExportValuesMultiPhase(result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "manopt_traj.csv");
}

/* ************************************************************************* */
void EvaluateAndExportBarrierResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, PenaltyItersDetails> &penalty_result,
    const std::string &scenario_folder, bool print_values) {
  size_t num_steps = vision60_multi_phase.numSteps();
  const Values &penalty_result_values = penalty_result.second.rbegin()->values;
  problem.eval_func(penalty_result_values);
  if (print_values) {
    IEVision60Robot::PrintValues(penalty_result_values, num_steps);
  }
  IEVision60Robot::ExportValuesMultiPhase(penalty_result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "barrier_traj.csv");
  penalty_result.first.exportFile(scenario_folder + "barrier_summary.csv");
}

/* ************************************************************************* */
void EvaluateAndExportInitValues(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::string &scenario_folder, bool print_values) {
  problem.eval_func(problem.initValues());
  if (print_values) {
    IEVision60Robot::PrintValues(problem.initValues(),
                                 vision60_multi_phase.numSteps());
  }
  IEVision60Robot::ExportValuesMultiPhase(problem.initValues(),
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "init_traj.csv");
}

/* ************************************************************************* */
void ExportCostEval(std::map<std::string, std::vector<double>> &iters_cost_eval,
                    const std::string &file_path) {
  std::ofstream cost_file;
  cost_file.open(file_path);
  cost_file << "iterations";
  for (const auto &[category_name, iters_cost_eval] : iters_cost_eval) {
    if (category_name != "iterations") {
      cost_file << "," << category_name;
    }
  }
  cost_file << "\n";
  size_t num_iters = iters_cost_eval.at("iterations").size();
  for (size_t i = 0; i < num_iters; i++) {
    cost_file << (size_t)iters_cost_eval.at("iterations").at(i);
    for (const auto &[category_name, iters_cost_eval] : iters_cost_eval) {
      if (category_name != "iterations") {
        cost_file << "," << iters_cost_eval.at(i);
      }
    }
    cost_file << "\n";
  }
  cost_file.close();
}

/* ************************************************************************* */
void ExportActiveConstraints(
    const std::vector<std::pair<size_t, std::vector<std::string>>>
        &iters_active_names,
    const std::string &file_path) {
  std::ofstream constraint_file;
  constraint_file.open(file_path);
  for (const auto &[iterations, active_names] : iters_active_names) {
    constraint_file << iterations;
    for (const auto &active_name : active_names) {
      constraint_file << "," << active_name;
    }
    constraint_file << "\n";
  }
  constraint_file.close();
}

/* ************************************************************************* */
std::vector<std::string> ActiveNames(const IEManifoldValues &manifolds) {
  std::vector<std::string> active_names;
  for (const auto &[key, manifold] : manifolds) {
    for (const auto &constraint_idx : manifold.activeIndices()) {
      const auto &constraint = manifold.iConstraints()->at(constraint_idx);
      active_names.push_back(constraint->name_tmp());
    }
  }
  return active_names;
}

/* ************************************************************************* */
void ExportOptimizationProgress(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::string &scenario_folder, const IELMItersDetails &iters_details) {
  std::string progress_folder = scenario_folder + "progress/";
  std::filesystem::create_directory(progress_folder);

  //// Export state and trial info.
  std::string manopt_state_file_path = progress_folder + "states_info.csv";
  std::string manopt_trial_file_path = progress_folder + "trials_info.csv";
  iters_details.exportFile(manopt_state_file_path, manopt_trial_file_path);

  auto classified_costs = vision60_multi_phase.classifiedCosts();
  std::map<std::string, std::vector<double>> iters_state_costs_eval;
  std::map<std::string, std::vector<double>> iters_trials_costs_eval;
  iters_state_costs_eval.insert({"iterations", std::vector<double>()});
  iters_trials_costs_eval.insert({"iterations", std::vector<double>()});
  for (const auto &[cost_category, graph] : classified_costs) {
    iters_state_costs_eval.insert({cost_category, std::vector<double>()});
    iters_trials_costs_eval.insert({cost_category, std::vector<double>()});
  }
  std::vector<std::pair<size_t, std::vector<std::string>>>
      iters_state_active_names;
  std::vector<std::pair<size_t, std::vector<std::string>>>
      iters_trials_active_names;

  for (const auto &iter_details : iters_details) {
    const auto &state = iter_details.state;
    Values state_values = state.baseValues();
    std::string state_str = "state_" + std::to_string(state.iterations);

    // Export state trajectory info
    IEVision60Robot::ExportValuesMultiPhase(
        state_values, vision60_multi_phase.phase_num_steps_,
        progress_folder + "traj_" + state_str + ".csv");

    // Export state values
    ExportValuesToFile(state_values,
                       progress_folder + "values_" + state_str + ".dat");

    // Record state cost info
    for (const auto &[cost_category, graph] : classified_costs) {
      iters_state_costs_eval.at(cost_category)
          .emplace_back(graph.error(state_values));
    }
    iters_state_costs_eval.at("iterations").emplace_back(state.iterations);

    // Record state active constraint info
    auto state_active_names = ActiveNames(state.manifolds);
    iters_state_active_names.emplace_back(state.iterations, state_active_names);

    for (size_t trial_id = 0; trial_id < iter_details.trials.size();
         trial_id++) {
      const auto &trial = iter_details.trials.at(trial_id);
      Values trial_values = state.baseValues();
      std::string trial_str = "trial_" + std::to_string(state.iterations) +
                              "_" + std::to_string(trial_id);

      // Export trial trajectory info
      IEVision60Robot::ExportValuesMultiPhase(
          trial_values, vision60_multi_phase.phase_num_steps_,
          progress_folder + "traj_" + trial_str + ".csv");

      // Export trial values
      ExportValuesToFile(trial_values,
                         progress_folder + "values_" + trial_str + ".dat");

      // Record trial cost info
      for (const auto &[cost_category, graph] : classified_costs) {
        iters_trials_costs_eval.at(cost_category)
            .emplace_back(graph.error(trial_values));
      }
      iters_trials_costs_eval.at("iterations").emplace_back(state.iterations);

      // Record trial active constraint info
      auto trial_active_names = ActiveNames(state.manifolds);
      iters_trials_active_names.emplace_back(state.iterations,
                                             trial_active_names);
    }
  }

  // Export states and trials cost evaluations.
  ExportCostEval(iters_state_costs_eval,
                 progress_folder + "states_cost_eval.csv");
  ExportCostEval(iters_trials_costs_eval,
                 progress_folder + "trials_cost_eval.csv");

  // Export states and trials active constraints.
  ExportActiveConstraints(iters_state_active_names,
                          progress_folder + "states_active_constraints.csv");
  ExportActiveConstraints(iters_trials_active_names,
                          progress_folder + "trials_active_constraints.csv");
}

} // namespace gtdynamics
