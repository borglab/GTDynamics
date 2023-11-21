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

#include "QuadrupedVerticalJump.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

using namespace gtdynamics;
using namespace gtsam;
using namespace quadruped_vertical_jump;

bool include_inequality = false;
std::string constraint_str = include_inequality ? "ie" : "e";
std::string scenario = "yetong08_" + constraint_str + "_quadruped_jump";
std::string scenario_folder = "../../data/" + scenario + "/";

typedef std::vector<
    std::pair<std::string, IEConstraintManifold::Params::shared_ptr>>
    IECM_PARAMS_LIST;

void RunRetractorBenchMark(const IEConsOptProblem &problem,
                           const IECM_PARAMS_LIST &iecm_params_list,
                           const IELMParams &ie_params) {
  std::filesystem::create_directory(scenario_folder);
  for (const auto &[exp_name, iecm_params] : iecm_params_list) {
    std::cout << exp_name << ":\n";

    // run experiment
    auto lm_result = OptimizeIELM(problem, ie_params, iecm_params);

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

    // export file
    const auto &iters_details = lm_result.second;
    std::string state_file_path = scenario_folder + exp_name + "_states.csv";
    std::string trial_file_path = scenario_folder + exp_name + "_trials.csv";
    iters_details.exportFile(state_file_path, trial_file_path);
  }
}

void RetractorBenchMark() {
  VerticalJumpParams params;
  params.include_inequalities = include_inequality;
  params.add_phase_prior = true;
  params.phase_prior_dt = std::vector<double>{0.025, 0.025};
  params.vision60_params.ad_basis_using_torques = false;
  auto vision60_multi_phase_a = GetVision60MultiPhase(params);
  params.vision60_params.ad_basis_using_torques = true;
  auto vision60_multi_phase_T = GetVision60MultiPhase(params);

  /// Create problem
  auto problem = CreateProblem(params);

  std::vector<std::tuple<std::string, std::string, std::string>> exp_settings;
  // exp_settings.emplace_back("Orthonormal", "Barrier", "all");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "cost");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "costscale");
  // // exp_settings.emplace_back("Orthonormal", "Barrier", "basisa");
  // exp_settings.emplace_back("Orthonormal", "Barrier", "basisT");

  exp_settings.emplace_back("Orthonormal", "Hierarchical", "all");
  exp_settings.emplace_back("Orthonormal", "Hierarchical", "cost");
  exp_settings.emplace_back("Orthonormal", "Hierarchical", "costscale");
  // exp_settings.emplace_back("Orthonormal", "Hierarchical", "basisa");
  exp_settings.emplace_back("Orthonormal", "Hierarchical", "basisT");

  // exp_settings.emplace_back("EliminationT", "Barrier", "all");
  // exp_settings.emplace_back("EliminationT", "Barrier", "cost");
  // exp_settings.emplace_back("EliminationT", "Barrier", "costscale");
  // // exp_settings.emplace_back("EliminationT", "Barrier", "basisa");
  // exp_settings.emplace_back("EliminationT", "Barrier", "basisT");

  // exp_settings.emplace_back("EliminationT", "Hierarchical", "all");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "cost");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "costscale");
  // // exp_settings.emplace_back("EliminationT", "Hierarchical", "basisa");
  // exp_settings.emplace_back("EliminationT", "Hierarchical", "basisT");

  IECM_PARAMS_LIST iecm_params_list;
  for (const auto &[basis_type, retractor_type, metric_type] : exp_settings) {
    auto iecm_params = std::make_shared<IEConstraintManifold::Params>();

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
    // retractor_params->lm_params.minModelFidelity = 0.5;
    retractor_params->check_feasible = true;
    retractor_params->feasible_threshold = 1e-3;
    retractor_params->prior_sigma = 0.1;
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
  IELMParams ie_params;
  ie_params.lm_params.setLinearSolverType("SEQUENTIAL_QR");
  ie_params.lm_params.setVerbosityLM("SUMMARY");
  ie_params.lm_params.setMaxIterations(100);
  ie_params.lm_params.setlambdaInitial(1e-2);
  ie_params.lm_params.setlambdaUpperBound(1e10);
  RunRetractorBenchMark(problem, iecm_params_list, ie_params);
}

int main(int argc, char **argv) {
  RetractorBenchMark();
  return 0;
}
