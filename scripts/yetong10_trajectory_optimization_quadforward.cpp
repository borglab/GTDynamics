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

#include "QuadrupedExpUtils.h"
#include "QuadrupedForwardJump.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_forward_jump;

bool include_inequality = true;
std::string constraint_str = include_inequality ? "ie" : "e";
std::string scenario = "yetong10_" + constraint_str + "_quadruped_jump";
std::string scenario_folder = "../../data/" + scenario + "/";

/* <=====================================================================> */
/* <========================== Create Problem ===========================> */
/* <=====================================================================> */
std::tuple<IEConsOptProblem, IEVision60RobotMultiPhase::shared_ptr, ForwardJumpParams>
CreateProblem() {
  std::filesystem::create_directory(scenario_folder);

  /* <=========== scenario setting ===========> */
  ForwardJumpParams params;
  params.vision60_params->eval_details = true;
  params.vision60_params->eval_collo_step = true;
  params.vision60_params->i_constraints_symmetry = true;
  params.phase_num_steps = std::vector<size_t>{20, 10, 20};
  params.phases_dt = std::vector<double>{0.01, 0.02, 0.02};
  params.init_values_include_i_constraints = true;
  params.init_values_ensure_feasible = true;

  /* <=========== costs ===========> */
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  params.vision60_params->include_cf_jerk_costs = true;
  params.vision60_params->include_symmetry_costs = true;
  params.vision60_params->collision_as_cost = true;

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
  params.vision60_params->include_phase_duration_limits = true;
  params.vision60_params->phases_min_dt =
      std::vector<double>{0.01, 0.001, 0.01};
  if (include_inequality) {
    params.vision60_params->include_friction_cone = true;
    params.vision60_params->include_joint_limits = true;
    // params.vision60_params->include_torque_limits = true;
    // params.vision60_params->include_collision_free_z = true;
    // params.vision60_params->include_collision_free_s = true;
  }

  /* <=========== create problem ===========> */

  return CreateProblem(params);
}

void TrajectoryOptimization() {
  auto [problem, vision60_multi_phase, params] = CreateProblem();
  // auto init_values = LoadValuesFromFile(scenario_folder + "manopt_values_cpy.dat");
  // problem.values_ = init_values;
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
  retractor_params->ensure_feasible = true;
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
  ie_params.lm_params.setMaxIterations(200);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  ie_params.show_active_costraints = true;

  /* <=========== optimize ===========> */
  auto ielm_result = OptimizeIELM(problem, ie_params, iecm_params);
  EvaluateAndExportIELMResult(problem, *vision60_multi_phase, ielm_result,
                              scenario_folder, false);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
