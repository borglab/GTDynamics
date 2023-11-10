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
#include "gtdynamics/imanifold/IERetractor.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <memory>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_vertical_jump;

bool include_inequality = false;

void RunRetractorBenchMark(
    const IEConsOptProblem &problem,
    const std::vector<std::pair<std::string, IERetractorCreator::shared_ptr>>
        &retractor_creators,
    const IELMParams &ie_params,
    const IEConstraintManifold::Params::shared_ptr &_ecm_params) {
  auto iecm_params =
      std::make_shared<IEConstraintManifold::Params>(*_ecm_params);

  for (const auto &[retractor_name, retractor_creator] : retractor_creators) {
    iecm_params->retractor_creator = retractor_creator;
    std::cout << retractor_name << ":\n";
    auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);
    // Values result_values = lm_result.second.back().state.baseValues();
    const auto &iters_details = lm_result.second;
    std::string state_file_path =
        "../../data/quadruped_ground_air_" + retractor_name + "_states.csv";
    std::string trial_file_path =
        "../../data/quadruped_ground_air_" + retractor_name + "_trials.csv";
    iters_details.exportFile(state_file_path, trial_file_path);
  }
}

void RetractorBenchMark() {
  /// Initialize vision60 robot
  auto vision60_params = GetVision60Params();
  vision60_params.ad_basis_using_torques = false;
  auto vision60_multi_phase_a = GetVision60MultiPhase(vision60_params);
  vision60_params.ad_basis_using_torques = true;
  auto vision60_multi_phase_T = GetVision60MultiPhase(vision60_params);

  /// Create problem
  auto problem =
      CreateProblem(vision60_multi_phase_a, include_inequality, true);

  LevenbergMarquardtParams lm_params;
  BarrierRetractor::Params retractor_params(lm_params, 0.1);

  auto metric_sigmas = std::make_shared<VectorValues>();
  BarrierRetractor::Params metric_retractor_params(lm_params, metric_sigmas);

  auto barrier_retractor_creator_no_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_a, retractor_params, false);

  auto barrier_retractor_creator_a_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_a, retractor_params, true);

  auto barrier_retractor_creator_T_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_T, retractor_params, true);

  auto barrier_retractor_creator_diagonal_metric =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_T, metric_retractor_params, false);

  auto hierarchical_retractor_creator_no_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_a, retractor_params, false);

  auto hierarchical_retractor_creator_a_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_a, retractor_params, true);

  auto hierarchical_retractor_creator_T_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_T, retractor_params, true);

  auto hierarchical_retractor_creator_diagonal_metric =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_T, metric_retractor_params, false);

  std::vector<std::pair<std::string, IERetractorCreator::shared_ptr>>
      retractor_creators{
          {"barrier_none", barrier_retractor_creator_no_basis_keys},
          {"barrier_a", barrier_retractor_creator_a_basis_keys},
          {"barrier_T", barrier_retractor_creator_T_basis_keys},
          {"barrier_metric", barrier_retractor_creator_diagonal_metric},
          {"hierarchical_none", hierarchical_retractor_creator_no_basis_keys},
          {"hierarchical_a", hierarchical_retractor_creator_a_basis_keys},
          {"hierarchical_T", hierarchical_retractor_creator_T_basis_keys},
          {"hierarchical_metric",
           hierarchical_retractor_creator_diagonal_metric}};

  IELMParams ie_params;
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(30);
  ie_params.lm_params.setlambdaInitial(1e-2);

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_with_new_constraints = true;
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->e_basis_creator =
      std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
          vision60_multi_phase_T, iecm_params->ecm_params->basis_params);

  RunRetractorBenchMark(problem, retractor_creators, ie_params, iecm_params);
}

int main(int argc, char **argv) {
  RetractorBenchMark();
  return 0;
}
