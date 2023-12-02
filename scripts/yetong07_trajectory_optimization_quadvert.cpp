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

  /* <=====================================================================> */
  /* <========================== Create Problem ===========================> */
  /* <=====================================================================> */

  /* <=========== scenario setting ===========> */
  VerticalJumpParams params;
  params.vision60_params->eval_details = true;
  params.vision60_params->eval_collo_step = true;

  /* <=========== costs ===========> */
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  // params.vision60_params->include_accel_penalty = true;
  // params.vision60_params->accel_panalty_threshold = 200;
  // params.phase_prior_dt = std::vector<double>{0.025, 0.025};

  params.vision60_params->sigma_des_pose = 1e-2;
  params.vision60_params->sigma_des_twist = 1e-2;
  params.vision60_params->sigma_actuation = 10;
  params.vision60_params->sigma_jerk = 10;
  params.vision60_params->sigma_q_col = 1e-2;
  params.vision60_params->sigma_v_col = 1e-2;
  params.vision60_params->sigma_twist_col = 1e-2;
  params.vision60_params->sigma_pose_col = 1e-2;

  /* <=========== inequality constraints ===========> */
  params.vision60_params->include_phase_duration_limits = true;
  params.vision60_params->phases_min_dt = std::vector<double>{0.015, 0.015};
  if (include_inequality) {
    params.vision60_params->include_friction_cone = true;
    params.vision60_params->include_joint_limits = true;
    params.vision60_params->include_torque_limits = true;
    params.vision60_params->include_collision_free_z = true;
  }
  auto vision60_multi_phase = GetVision60MultiPhase(params);

  /* <=========== create problem ===========> */
  auto problem = CreateProblem(params);
  EvaluateAndExportInitValues(problem, vision60_multi_phase, scenario_folder);

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
  // ie_params.show_active_costraints = true;

  /* <=========== optimzie ===========> */
  auto ielm_result = OptimizeIELM(problem, ie_params, iecm_params);
  EvaluateAndExportIELMResult(problem, vision60_multi_phase, ielm_result,
                              scenario_folder, true);

  /* <=====================================================================> */
  /* <======================== Optimize Barrier ===========================> */
  /* <=====================================================================> */
  // BarrierParameters barrier_params;
  // barrier_params.verbose = true;
  // barrier_params.initial_mu = 1e0;
  // barrier_params.num_iterations = 10;
  // auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);
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
