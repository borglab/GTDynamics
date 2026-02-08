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
using namespace quadruped_forward_jump;

bool include_inequality = true;
std::string constraint_str = include_inequality ? "ie" : "e";
std::string scenario = "yetong10_" + constraint_str + "_quadruped_jump";
std::string scenario_folder = "../../data/" + scenario + "/";
bool log_results = true;

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
  params.forward_distance = 1.5;
  params.init_values_include_i_constraints = true;
  params.init_values_ensure_feasible = true;
  params.phase_num_steps = std::vector<size_t>{20, 10, 20};
  params.phases_dt = std::vector<double>{0.01, 0.02, 0.02};
  params.vision60_params->phases_min_dt =
      std::vector<double>{0.01, 0.001, 0.01};

  params.vision60_params->eval_details = true;
  params.vision60_params->eval_collo_step = true;
  params.vision60_params->i_constraints_symmetry = true;
  params.vision60_params->terrain_height_function =
      IEVision60Robot::sinHurdleTerrainFunc(0.75, 0.3, 0.2);
  params.vision60_params->step_div_ratio = 0.5;
  params.vision60_params->state_cost_values =
      DesValues(params.phase_num_steps, Point3(params.forward_distance, 0, 0));
  std::tie(params.vision60_params->state_cost_points,
           params.vision60_params->state_cost_point_vels) =
      DesPoints(params.phase_num_steps, Point3(params.forward_distance, 0, 0));

  /* <=========== costs ===========> */
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  params.vision60_params->include_cf_jerk_costs = true;
  params.vision60_params->include_symmetry_costs = true;
  params.vision60_params->include_collision_free_z_inter_cost = true;

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
  params.vision60_params->tol_cf = 1e-3;
  params.vision60_params->jerk_cost_option = JERK_DIV_DT;
  params.vision60_params->dt_threshold = 0.02;

  /* <=========== inequality constraints ===========> */
  if (include_inequality) {
    params.vision60_params->include_phase_duration_limits = true;
    params.vision60_params->include_friction_cone = true;
    params.vision60_params->include_joint_limits = true;
    // params.vision60_params->include_torque_limits = true;
    params.vision60_params->include_collision_free_z = true;
  } else {
    params.vision60_params->phase_duration_limit_as_cost = true;
    params.vision60_params->friction_cone_as_cost = true;
    params.vision60_params->joint_limits_as_cost = true;
    // params.vision60_params->torque_limits_as_cost = true;
    params.vision60_params->collision_as_cost = true;
  }

  /* <=========== create problem ===========> */
  auto vision60_multi_phase =
      GetVision60MultiPhase(params.vision60_params, params.phase_num_steps);

  /// Constraints
  EqualityConstraints e_constraints = vision60_multi_phase->eConstraints();
  InequalityConstraints i_constraints = vision60_multi_phase->iConstraints();
  NonlinearFactorGraph costs = vision60_multi_phase->costs();

  /// Initial Values
  auto init_values =
      InitValuesTrajectory(*vision60_multi_phase, params.phases_dt,
                           params.init_values_include_i_constraints,
                           params.init_values_ensure_feasible);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = vision60_multi_phase->costsEvalFunc();
  return {problem, vision60_multi_phase, params};
}

void TrajectoryOptimization() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();
  // auto init_values = LoadValuesFromFile(scenario_folder +
  // "manopt_values_cpy.dat"); problem.values_ = init_values;
  EvaluateAndExportInitValues(problem, *vision60_multi_phase, scenario_folder);

  /* <=====================================================================> */
  /* <========================== Optimize IELM ============================> */
  /* <=====================================================================> */
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_build_from_scratch = false;

  /* <=========== retractor ===========> */
  auto retractor_params = std::make_shared<IERetractorParams>();
  retractor_params->lm_params = LevenbergMarquardtParams();
  retractor_params->lm_params.setlambdaUpperBound(1e10);
  retractor_params->lm_params.setAbsoluteErrorTol(1e-10);
  retractor_params->check_feasible = true;
  retractor_params->ensure_feasible = true;
  retractor_params->feasible_threshold = 1e-5;
  retractor_params->prior_sigma = 1e-1;
  retractor_params->use_varying_sigma = true;
  // retractor_params->scale_varying_sigma = true;
  retractor_params->metric_sigmas = std::make_shared<VectorValues>();

  auto barrier_params = std::make_shared<PenaltyParameters>();
  // barrier_params->lm_params = params_->lm_params;
  barrier_params->initial_mu = 10.0;
  barrier_params->mu_increase_rate = 10.0;
  barrier_params->num_iterations = 2;
  auto lm_params1 = retractor_params->lm_params;
  auto lm_params2 = retractor_params->lm_params;
  // lm_params1.setMaxIterations(20);
  // lm_params1.setVerbosityLM("SUMMARY");
  barrier_params->iters_lm_params = std::vector<LevenbergMarquardtParams>();
  for (size_t i = 0; i < barrier_params->num_iterations - 1; i++) {
    barrier_params->iters_lm_params.push_back(lm_params1);
  }
  barrier_params->iters_lm_params.push_back(lm_params2);
  retractor_params->penalty_params = barrier_params;

  // iecm_params->retractor_creator =
  //     std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
  //         vision60_multi_phase, retractor_params, false);
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase, retractor_params, false);

  /* <=========== t-space basis ===========> */
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();

  /* <=========== IELM params ===========> */
  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(200);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  // ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  ie_params.iqp_max_iters = 100;
  ie_params.show_active_constraints = true;
  ie_params.active_constraints_group_as_categories = true;

  /* <=========== optimize ===========> */
  auto ielm_result = OptimizeIE_CMCOptLM(problem, ie_params, iecm_params);
  EvaluateAndExportIELMResult(problem, *vision60_multi_phase, ielm_result.second,
                              scenario_folder, false);
  ExportOptimizationProgress(*vision60_multi_phase, scenario_folder,
                             ielm_result.second);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
