#pragma once
#include "gtdynamics/utils/GraphUtils.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

namespace gtsam {
inline void EvaluateAndExportIELMResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, IELMItersDetails> &ielm_result,
    const std::string &scenario_folder, bool print_values = false,
    bool print_iter_details = false) {
  size_t num_steps = vision60_multi_phase.numSteps();

  //// Visualize optimization progress
  Values result_values = ielm_result.second.back().state.baseValues();
  if (print_iter_details) {
    for (const auto &iter_details : ielm_result.second) {
      IEOptimizer::PrintIterDetails(
          iter_details, num_steps, false, IEVision60Robot::PrintValues,
          IEVision60Robot::PrintDelta, gtdynamics::GTDKeyFormatter);
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
      double dt = values.atDouble(gtdynamics::PhaseKey(phase_idx));
      std::cout << "\t" << dt;
    }
    std::cout << "\n";
  }

  //// Export final optimized trajectory.
  ExportValuesToFile(result_values, scenario_folder + "manopt_values.dat");
  IEVision60Robot::ExportValuesMultiPhase(result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "manopt_traj.csv");

  //// Export the optimization progress.
  std::string manopt_state_file_path = scenario_folder + "manopt_states.csv";
  std::string manopt_trial_file_path = scenario_folder + "manopt_trials.csv";
  ielm_result.second.exportFile(manopt_state_file_path, manopt_trial_file_path);
}

inline void EvaluateAndExportBarrierResult(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::pair<IEResultSummary, BarrierItersDetail> &barrier_result,
    const std::string &scenario_folder, bool print_values = false) {
  size_t num_steps = vision60_multi_phase.numSteps();
  const Values &barrier_result_values = barrier_result.second.rbegin()->values;
  problem.eval_func(barrier_result_values);
  if (print_values) {
    IEVision60Robot::PrintValues(barrier_result_values, num_steps);
  }
  IEVision60Robot::ExportValuesMultiPhase(barrier_result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "barrier_traj.csv");
  barrier_result.first.exportFile(scenario_folder + "barrier_summary.csv");
  barrier_result.first.exportFileWithMu(scenario_folder +
                                        "barrier_summary_outerloop.csv");
}

inline void EvaluateAndExportInitValues(
    const IEConsOptProblem &problem,
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::string &scenario_folder, bool print_values = false) {
  problem.eval_func(problem.initValues());
  if (print_values) {
    IEVision60Robot::PrintValues(problem.initValues(),
                                 vision60_multi_phase.numSteps());
  }
  IEVision60Robot::ExportValuesMultiPhase(problem.initValues(),
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "init_traj.csv");
}

} // namespace gtsam
