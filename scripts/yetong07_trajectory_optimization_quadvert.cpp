/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include "QuadrupedVerticalJump.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_vertical_jump;

bool include_inequality = false;

void TrajectoryOptimization() {
  std::string constraint_str = include_inequality ? "ie" : "e";
  std::string scenario = "yetong07_" + constraint_str + "_quadruped_jump";
  std::string scenario_folder = "../../data/" + scenario + "/";
  std::filesystem::create_directory(scenario_folder);

  /// Initialize vision60 robot
  auto vision60_params = GetVision60Params();
  auto vision60_multi_phase = GetVision60MultiPhase(vision60_params);
  size_t num_steps = vision60_multi_phase.phase_num_steps_[0] +
                     vision60_multi_phase.phase_num_steps_[1];

  /// Create problem
  auto problem = CreateProblem(vision60_multi_phase, include_inequality, true);
  // IEVision60Robot::PrintValues(problem.initValues(), num_steps);
  problem.eval_func(problem.initValues());
  IEVision60Robot::ExportValues(problem.initValues(), num_steps,
                                scenario_folder + "init_traj_viz.csv");
  IEVision60Robot::ExportValuesMultiPhase(problem.initValues(),
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "init_traj.csv");

  /// optimize IELM
  // Parameters
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_with_new_constraints = true;
  iecm_params->ecm_params->basis_params->setFixVars();

  BarrierRetractor::Params retractor_params;
  retractor_params.lm_params = LevenbergMarquardtParams();
  // retractor_params.lm_params.minModelFidelity = 0.5;
  retractor_params.check_feasible = true;
  retractor_params.feasible_threshold = 1e-3;
  retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase, retractor_params, false);
  // iecm_params->retractor_creator =
  //     std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
  //         vision60_multi_phase, retractor_params, true);
  iecm_params->e_basis_creator =
      std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
          vision60_multi_phase, iecm_params->ecm_params->basis_params);

  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(30);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-1);
  auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);

  // problem = CreateProblem(vision60_multi_phase, include_inequality, false);
  // problem.values_ = lm_result.second.back().state.baseValues();
  // lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);

  Values result_values = lm_result.second.back().state.baseValues();
  // for (const auto &iter_details : lm_result.second) {
  //   IEOptimizer::PrintIterDetails(
  //       iter_details, num_steps, false, IEVision60Robot::PrintValues,
  //       IEVision60Robot::PrintDelta, gtdynamics::GTDKeyFormatter);
  // }
  problem.eval_func(result_values);
  IEVision60Robot::PrintValues(result_values, num_steps);
  IEVision60Robot::ExportValues(result_values, num_steps,
                                scenario_folder + "manopt_traj_viz.csv");
  IEVision60Robot::ExportValuesMultiPhase(result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "manopt_traj.csv");
  lm_result.first.exportFile(scenario_folder + "manopt_summary.csv");
  std::string manopt_state_file_path = scenario_folder + "manopt_states.csv";
  std::string manopt_trial_file_path = scenario_folder + "manopt_trials.csv";
  lm_result.second.exportFile(manopt_state_file_path, manopt_trial_file_path);

  /// Optimize Barrier
  BarrierParameters barrier_params;
  barrier_params.verbose = true;
  barrier_params.initial_mu = 1e0;
  barrier_params.num_iterations = 10;
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);
  const Values &barrier_result_values = barrier_result.second.rbegin()->values;
  problem.eval_func(barrier_result_values);
  // IEVision60Robot::PrintValues(barrier_result_values, num_steps);
  IEVision60Robot::ExportValues(barrier_result_values, num_steps,
                                scenario_folder + "barrier_traj_viz.csv");
  IEVision60Robot::ExportValuesMultiPhase(barrier_result_values,
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "barrier_traj.csv");
  barrier_result.first.exportFile(scenario_folder + "barrier_summary.csv");
  barrier_result.first.exportFileWithMu(scenario_folder +
                                        "barrier_summary_outerloop.csv");

  barrier_result.first.printLatex(std::cout);
  lm_result.first.printLatex(std::cout);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
