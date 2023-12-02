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

#include "QuadrupedForwardJump.h"
#include "QuadrupedExpUtils.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_forward_jump;

bool include_inequality = true;



void TrajectoryOptimization() {
  std::string constraint_str = include_inequality ? "ie" : "e";
  std::string scenario = "yetong10_" + constraint_str + "_quadruped_jump";
  std::string scenario_folder = "../../data/" + scenario + "/";
  std::filesystem::create_directory(scenario_folder);

  ForwardJumpParams params;
  params.vision60_params->ad_basis_using_torques = true;

  // costs
  params.vision60_params->include_collocation_costs = true;
  params.vision60_params->include_actuation_costs = true;
  params.vision60_params->include_state_costs = true;
  params.vision60_params->include_jerk_costs = true;
  params.vision60_params->include_accel_penalty = true;
  params.vision60_params->accel_panalty_threshold = 30;
  params.vision60_params->collision_as_cost = true;

  // constriants
  // params.vision60_params->boundary_constrain_a = false;

  // params.vision60_params->include_phase_duration_prior_costs = true;
  // params.vision60_params->phase_prior_dt = std::vector<double>{0.025, 0.02,
  // 0.02};

  params.vision60_params->include_phase_duration_limits = true;
  params.vision60_params->phases_min_dt =
      std::vector<double>{0.01, 0.005, 0.015};

  if (include_inequality) {
    // params.vision60_params->include_friction_cone = true;
    // params.vision60_params->include_joint_limits = true;
    // params.vision60_params->include_torque_limits = true;
    // params.vision60_params->include_collision_free_z = true;
  }

  auto vision60_multi_phase =
      GetVision60MultiPhase(params.vision60_params, params.phase_num_steps);
  size_t num_steps = vision60_multi_phase.numSteps();

  /// Create problem
  auto problem = CreateProblem(params);
  // IEVision60Robot::PrintValues(problem.initValues(), num_steps);
  problem.eval_func(problem.initValues());
  IEVision60Robot::ExportValuesMultiPhase(problem.initValues(),
                                          vision60_multi_phase.phase_num_steps_,
                                          scenario_folder + "init_traj.csv");

  /// optimize IELM
  // Parameters
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_build_from_scratch = false;

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
  // iecm_params->e_basis_creator =
  //     std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
  //         vision60_multi_phase);
  iecm_params->e_basis_creator = OrthonormalBasisCreator::CreateSparse();

  IELMParams ie_params;
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(100);
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  auto ielm_result = OptimizeIELM(problem, ie_params, iecm_params);

  EvaluateAndExportIELMResult(problem, vision60_multi_phase, ielm_result,
                              scenario_folder, true);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}
