/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  retractor_benchmark_quadvert.cpp
 * @brief Tspace and Retractor benchmarking in a quadruped vertical jump
 * trajectory optimization problem.
 * @author Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmarkIE.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_vertical_jump;

bool include_inequality = false;
std::string constraint_str = include_inequality ? "ie" : "e";
std::string scenario = "yetong08_" + constraint_str + "_quadruped_jump";
std::string scenario_folder = "../../data/" + scenario + "/";

typedef std::vector<std::tuple<std::string, std::string, std::string>>
    EXP_SETTING_LIST;
typedef std::vector<
    std::pair<std::string, IEConstraintManifold::Params::shared_ptr>>
    IECM_PARAMS_LIST;

/* <=====================================================================> */
/* <======================= Benchmark Experiment ========================> */
/* <=====================================================================> */
void RunRetractorBenchMark(
    const IEConsOptProblem &problem, const IECM_PARAMS_LIST &iecm_params_list,
    const IELMParams &ie_params,
    const IEVision60RobotMultiPhase &vision60_multi_phase) {
  std::filesystem::create_directory(scenario_folder);
  for (const auto &[exp_name, iecm_params] : iecm_params_list) {
    std::cout << exp_name << ":\n";

    // run experiment
    auto lm_result = OptimizeIE_CMCOptLM(problem, ie_params, iecm_params);

    // print result and evaluation
    Values result_values = lm_result.second.back().state.baseValues();
    problem.eval_func(result_values);
    for (const auto &iter_details : lm_result.second) {
      Values values = iter_details.state.baseValues();
      double dt1 = values.atDouble(PhaseKey(0));
      double dt2 = values.atDouble(PhaseKey(1));
      std::cout << iter_details.state.iterations << "\t" << dt1 << "\t" << dt2
                << "\n";
    }

    //// Export final optimized trajectory.
    ExportValuesToFile(result_values,
                       scenario_folder + exp_name + "_values.dat");
    IEVision60Robot::ExportValuesMultiPhase(
        result_values, vision60_multi_phase.phase_num_steps_,
        scenario_folder + exp_name + "_traj.csv");

    //// Export the optimization progress.
    const auto &iters_details = lm_result.second;
    std::string state_file_path = scenario_folder + exp_name + "_states.csv";
    std::string trial_file_path = scenario_folder + exp_name + "_trials.csv";
    iters_details.exportFile(state_file_path, trial_file_path);
  }
}

bool FileExists(const std::string &file_path) {
  return std::filesystem::exists(file_path);
}

/* <=====================================================================> */
/* <======================= Benchmark Experiment ========================> */
/* <=====================================================================> */
void RunMultiStageOptimization(
    IEConsOptProblem problem, const IECM_PARAMS_LIST &iecm_params_list,
    const IELMParams &ie_params,
    const IEVision60RobotMultiPhase &vision60_multi_phase) {
  std::filesystem::create_directory(scenario_folder);
  std::string exp_name = "";

  for (const auto &[exp_name_stage, iecm_params] : iecm_params_list) {
    if (exp_name == "") {
      exp_name = exp_name_stage;
    } else {
      exp_name += "_" + exp_name_stage;
    }
    std::cout << exp_name << ":\n";

    std::string values_file_path = scenario_folder + exp_name + "_values.dat";
    if (FileExists(values_file_path)) {
#if GTSAM_ENABLE_BOOST_SERIALIZATION
      problem.values_ = LoadValuesFromFile(values_file_path);
      continue;
#endif
    }

    // run experiment
    auto lm_result = OptimizeIE_CMCOptLM(problem, ie_params, iecm_params);

    // print result and evaluation
    Values result_values = lm_result.second.back().state.baseValues();
    problem.values_ = result_values;
    problem.eval_func(result_values);
    for (const auto &iter_details : lm_result.second) {
      Values values = iter_details.state.baseValues();
      double dt1 = values.atDouble(PhaseKey(0));
      double dt2 = values.atDouble(PhaseKey(1));
      std::cout << iter_details.state.iterations << "\t" << dt1 << "\t" << dt2
                << "\n";
    }

    //// Export final optimized trajectory.
    ExportValuesToFile(result_values, values_file_path);
    IEVision60Robot::ExportValuesMultiPhase(
        result_values, vision60_multi_phase.phase_num_steps_,
        scenario_folder + exp_name + "_traj.csv");

    //// Export the optimization progress.
    const auto &iters_details = lm_result.second;
    std::string state_file_path = scenario_folder + exp_name + "_states.csv";
    std::string trial_file_path = scenario_folder + exp_name + "_trials.csv";
    iters_details.exportFile(state_file_path, trial_file_path);
  }
}

/* <=====================================================================> */
/* <========================== Create Problem ===========================> */
/* <=====================================================================> */
std::tuple<IEConsOptProblem, IEVision60RobotMultiPhase::shared_ptr, JumpParams>
CreateProblem() {
  std::filesystem::create_directory(scenario_folder);

  /* <=========== scenario setting ===========> */
  JumpParams params;
  params.phase_num_steps = std::vector<size_t>{20, 10};
  params.phases_dt = std::vector<double>{0.01, 0.02};
  params.vision60_params->phases_min_dt = std::vector<double>{0.01, 0.005};
  params.vision60_params->eval_details = true;
  params.vision60_params->eval_collo_step = true;
  params.vision60_params->i_constraints_symmetry = true;
  params.vision60_params->state_cost_values = DesValues(
      params.phase_num_steps, Pose3(Rot3::Identity(), Point3(0, 0, 0.8)));

  /* <=========== costs ===========> */
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  params.vision60_params->include_cf_jerk_costs = true;
  params.vision60_params->include_symmetry_costs = true;
  // params.vision60_params->include_accel_penalty = true;
  // params.vision60_params->accel_panalty_threshold = 200;
  // params.phase_prior_dt = std::vector<double>{0.025, 0.025};

  params.vision60_params->sigma_des_pose = 2e-3;
  params.vision60_params->sigma_des_twist = 1e-2;
  params.vision60_params->sigma_actuation = 10;
  params.vision60_params->sigma_jerk = 3;
  params.vision60_params->sigma_q_col = 1e-2;
  params.vision60_params->sigma_v_col = 1e-2;
  params.vision60_params->sigma_twist_col = 2e-3;
  params.vision60_params->sigma_pose_col = 2e-3;
  params.vision60_params->sigma_cf_jerk = 1e1;
  params.vision60_params->sigma_symmetry = 1e-1;
  params.vision60_params->cf_jerk_threshold = 30;

  /* <=========== inequality constraints ===========> */
  if (include_inequality) {
    params.vision60_params->include_phase_duration_limits = true;
    params.vision60_params->include_friction_cone = true;
    params.vision60_params->include_joint_limits = true;
    params.vision60_params->include_torque_limits = true;
    params.vision60_params->include_collision_free_z = true;
  } else {
    params.vision60_params->phase_duration_limit_as_cost = true;
    params.vision60_params->friction_cone_as_cost = true;
    params.vision60_params->joint_limits_as_cost = true;
    params.vision60_params->torque_limits_as_cost = true;
    params.vision60_params->collision_as_cost = true;
  }

  /* <=========== create problem ===========> */
  auto vision60_multi_phase =
      GetVision60MultiPhase(params.vision60_params, params.phase_num_steps);

  /// Constraints and costs.
  EqualityConstraints e_constraints = vision60_multi_phase->eConstraints();
  InequalityConstraints i_constraints = vision60_multi_phase->iConstraints();
  NonlinearFactorGraph costs = vision60_multi_phase->costs();

  /// Initial Values
  // auto init_values =
  //     InitValuesTrajectory(vision60_multi_phase, params.phases_dt, 15, 5,
  //                          params.init_values_with_trapezoidal);
  auto init_values =
      InitValuesTrajectoryInfeasible(*vision60_multi_phase, params.phases_dt);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = vision60_multi_phase->costsEvalFunc();
  return {problem, vision60_multi_phase, params};
}

/* <=====================================================================> */
/* <====================== Experiment iecm_params =======================> */
/* <=====================================================================> */
IECM_PARAMS_LIST ConstructExpIECMParams(const EXP_SETTING_LIST &exp_settings,
                                        const JumpParams &params) {
  auto new_params = params;
  new_params.vision60_params =
      std::make_shared<IEVision60Robot::Params>(*params.vision60_params);
  new_params.vision60_params->ad_basis_using_torques = false;
  auto vision60_multi_phase_a = GetVision60MultiPhase(
      new_params.vision60_params, new_params.phase_num_steps);
  new_params.vision60_params->ad_basis_using_torques = true;
  auto vision60_multi_phase_T = GetVision60MultiPhase(
      new_params.vision60_params, new_params.phase_num_steps);

  IECM_PARAMS_LIST iecm_params_list;
  for (const auto &[basis_type, retractor_type, metric_type] : exp_settings) {
    auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
    iecm_params->e_basis_build_from_scratch = false;

    /// Tspace Basis
    if (basis_type == "Orthonormal") {
      iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();
    } else if (basis_type == "EliminationT") {
      iecm_params->e_basis_creator =
          std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
              vision60_multi_phase_T);
    } else if (basis_type == "Eliminationa") {
      iecm_params->e_basis_creator =
          std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
              vision60_multi_phase_a);
    }

    /// Retractor params
    auto retractor_params = std::make_shared<IERetractorParams>();
    retractor_params->lm_params = LevenbergMarquardtParams();
    retractor_params->check_feasible = true;
    retractor_params->feasible_threshold = 1e-3;
    retractor_params->prior_sigma = 1;
    if ((metric_type == "cost") || (metric_type == "costscale")) {
      retractor_params->use_varying_sigma = true;
      retractor_params->metric_sigmas = std::make_shared<VectorValues>();
    }
    if (metric_type == "costscale") {
      retractor_params->scale_varying_sigma = true;
    }
    /// Retractor
    if (retractor_type == "Barrier") {
      if (metric_type == "basisT") {
        iecm_params->retractor_creator =
            std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
                vision60_multi_phase_T, retractor_params, true);
      } else if (metric_type == "basisa") {
        iecm_params->retractor_creator =
            std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
                vision60_multi_phase_a, retractor_params, true);
      } else {
        iecm_params->retractor_creator =
            std::make_shared<BarrierRetractorCreator>(retractor_params);
      }
    } else if (retractor_type == "Hierarchical") {
      if (metric_type == "basisT") {
        iecm_params->retractor_creator =
            std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
                vision60_multi_phase_T, retractor_params, true);
      } else if (metric_type == "basisa") {
        iecm_params->retractor_creator =
            std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
                vision60_multi_phase_a, retractor_params, true);
      } else {
        iecm_params->retractor_creator =
            std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
                vision60_multi_phase_T, retractor_params, false);
      }
    }
    std::string exp_name =
        basis_type + "_" + retractor_type + "_" + metric_type;
    iecm_params_list.emplace_back(exp_name, iecm_params);
  }

  return iecm_params_list;
}

/* <=====================================================================> */
/* <========================= Benchmark Script ==========================> */
/* <=====================================================================> */
void RetractorBenchMark() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();

  /* <=========== IELM params ===========> */
  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(100);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  ie_params.show_active_constraints = true;
  ie_params.active_constraints_group_as_categories = true;

  /* <=========== experimental settings ===========> */
  EXP_SETTING_LIST exp_settings;
  // exp_settings.emplace_back("Orthonormal", "Barrier", "all");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "cost");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "costscale");
  // // exp_settings.emplace_back("Orthonormal", "Barrier", "basisa");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "basisT");

  // exp_settings.emplace_back("Orthonormal", "Hierarchical", "all");
  // exp_settings.emplace_back("Orthonormal", "Hierarchical", "cost");
  // exp_settings.emplace_back("Orthonormal", "Hierarchical", "costscale");
  // // exp_settings.emplace_back("Orthonormal", "Hierarchical", "basisa");
  // exp_settings.emplace_back("Orthonormal", "Hierarchical", "basisT");

  // exp_settings.emplace_back("EliminationT", "Barrier", "all");
  exp_settings.emplace_back("EliminationT", "Barrier", "cost");
  // exp_settings.emplace_back("EliminationT", "Barrier", "costscale");
  // // exp_settings.emplace_back("EliminationT", "Barrier", "basisa");
  exp_settings.emplace_back("EliminationT", "Barrier", "basisT");

  // exp_settings.emplace_back("EliminationT", "Hierarchical", "all");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "cost");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "costscale");
  // // exp_settings.emplace_back("EliminationT", "Hierarchical", "basisa");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "basisT");
  auto iecm_params_list = ConstructExpIECMParams(exp_settings, params);

  /* <=========== Run Benchmark ===========> */
  RunRetractorBenchMark(problem, iecm_params_list, ie_params,
                        *vision60_multi_phase);
}

/* <=====================================================================> */
/* <================= Multi-stage Optimization Script ===================> */
/* <=====================================================================> */
void MultiStageOptimization() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();

  /* <=========== IELM params ===========> */
  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(100);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-5);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  ie_params.show_active_constraints = true;
  ie_params.active_constraints_group_as_categories = true;

  /* <=========== experimental settings ===========> */
  EXP_SETTING_LIST exp_settings1;
  exp_settings1.emplace_back("EliminationT", "Barrier", "cost");
  exp_settings1.emplace_back("EliminationT", "Barrier", "cost");
  auto iecm_params_list1 = ConstructExpIECMParams(exp_settings1, params);

  EXP_SETTING_LIST exp_settings2;
  exp_settings2.emplace_back("EliminationT", "Barrier", "cost");
  exp_settings2.emplace_back("Orthonormal", "Barrier", "cost");
  auto iecm_params_list2 = ConstructExpIECMParams(exp_settings2, params);

  /* <=========== Run multi-stage optimization ===========> */
  // RunMultiStageOptimization(problem, iecm_params_list1, ie_params,
  //                           *vision60_multi_phase);

  RunMultiStageOptimization(problem, iecm_params_list2, ie_params,
                            *vision60_multi_phase);
}

int main(int argc, char **argv) {
  // RetractorBenchMark();
  MultiStageOptimization();
  return 0;
}
