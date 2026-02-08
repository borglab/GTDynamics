/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot vertical jump.
 * @author Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmarkIE.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_vertical_jump;

bool include_inequality = true;
bool two_stage_optimization = true;
bool log_results = true;

std::string constraint_str = include_inequality ? "ie" : "e";
std::string scenario = "yetong07_" + constraint_str + "_quadruped_jump";
std::string scenario_folder = "../../data/" + scenario + "/";

/* <=====================================================================> */
/* <========================== Create Problem ===========================> */
/* <=====================================================================> */
std::tuple<IEConsOptProblem, IEVision60RobotMultiPhase::shared_ptr, JumpParams>
CreateProblem() {
  if (log_results) {
    std::filesystem::create_directory(scenario_folder);
  }

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

void TrajectoryOptimization() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();
  EvaluateAndExportInitValues(problem, *vision60_multi_phase, scenario_folder);

  /* <=====================================================================> */
  /* <========================== Optimize IELM ============================> */
  /* <=====================================================================> */
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_build_from_scratch = false;

  /* <=========== retractor ===========> */
  auto retractor_params = std::make_shared<IERetractorParams>();
  retractor_params->lm_params = LevenbergMarquardtParams();
  retractor_params->check_feasible = true;
  retractor_params->feasible_threshold = 1e-3;
  retractor_params->prior_sigma = 0.1;
  // retractor_params->use_varying_sigma = true;
  // retractor_params->scale_varying_sigma = true;
  // retractor_params->metric_sigmas = std::make_shared<VectorValues>();
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase, retractor_params, false);
  // iecm_params->retractor_creator =
  //     std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
  //         vision60_multi_phase, retractor_params, false);

  /* <=========== t-space basis ===========> */
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();

  /* <=========== IELM params ===========> */
  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(100);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  ie_params.show_active_constraints = true;
  ie_params.active_constraints_group_as_categories = true;

  /* <=========== optimize ===========> */
  auto ielm_result = OptimizeIE_CMCOptLM(problem, ie_params, iecm_params);
  EvaluateAndExportIELMResult(problem, *vision60_multi_phase, ielm_result.second,
                              scenario_folder, false);

  /* <=========== 2nd stage optimization ===========> */
  // if (two_stage_optimization) {
  //   JumpParams new_params = params;
  //   new_params.vision60_params =
  //       std::make_shared<IEVision60Robot::Params>(*params.vision60_params);
  //   new_params.vision60_params->sigma_des_pose /= 5;
  //   new_params.vision60_params->sigma_des_twist /= 5;
  //   new_params.vision60_params->sigma_q_col /= 5;
  //   new_params.vision60_params->sigma_v_col /= 5;
  //   new_params.vision60_params->sigma_twist_col /= 5;
  //   new_params.vision60_params->sigma_pose_col /= 5;
  //   // new_params.vision60_params->include_actuation_costs = false;
  //   // new_params.vision60_params->include_jerk_costs = false;
  //   // new_params.vision60_params->include_state_costs = false;
  //   auto new_problem = std::get<0>(CreateProblem(new_params));
  //   new_problem.values_ = ielm_result.second.back().state.baseValues();
  //   EvaluateAndExportInitValues(new_problem, *vision60_multi_phase,
  //                               scenario_folder);
  //   auto new_ielm_result = OptimizeIE_CMCOptLM(new_problem, ie_params, iecm_params);
  //   EvaluateAndExportIELMResult(problem, *vision60_multi_phase,
  //   new_ielm_result,
  //                               scenario_folder, false);
  // }

  /* <=====================================================================> */
  /* <======================== Optimize Barrier ===========================> */
  /* <=====================================================================> */
  // auto barrier_params = std::make_shared<PenaltyParameters>();
  // barrier_params->verbose = true;
  // barrier_params->initial_mu = 1e0;
  // barrier_params->num_iterations = 10;
  // auto barrier_result = OptimizeIE_Penalty(problem, barrier_params);
  // EvaluateAndExportBarrierResult(problem, vision60_multi_phase,
  // barrier_result,
  //                                scenario_folder);

  // barrier_result.first.printLatex(std::cout);
  // lm_result.first.printLatex(std::cout);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
