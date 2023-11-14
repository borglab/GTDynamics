/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testManifoldOpt_so2.cpp
 * @brief Test manifold optimizer with SO(2) manifold.
 * @author Yetong Zhang
 */

#include "gtdynamics/imanifold/IERetractor.h"
#include "gtdynamics/manifold/TspaceBasis.h"
#include "gtdynamics/optimizer/InequalityConstraint.h"
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/expressions.h>
#include <memory>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

Key x1_key = 1;
Key x2_key = 2;

double dist_square_func(const double &x1, const double &x2,
                        gtsam::OptionalJacobian<1, 1> H1 = nullptr,
                        gtsam::OptionalJacobian<1, 1> H2 = nullptr) {
  if (H1)
    *H1 << 2 * x1;
  if (H2)
    *H2 << 2 * x2;
  return x1 * x1 + x2 * x2;
}

void SaveSummary(const Point2 &init_point, const Point2 &goal_point,
                 const Vector2 &cost_sigmas, int num_iters,
                 const std::string &folder_path) {
  std::string file_path = folder_path + "summary.txt";
  std::ofstream file;
  file.open(file_path);
  file << init_point.x() << " " << init_point.y() << "\n";
  file << goal_point.x() << " " << goal_point.y() << "\n";
  file << cost_sigmas(0) << " " << cost_sigmas(1) << "\n";
  file << num_iters << "\n";
  file.close();
}

void SaveState(const IELMState &state, int iter_id,
               const std::string &folder_path, int num_trials) {
  double x1 = state.baseValues().atDouble(x1_key);
  double x2 = state.baseValues().atDouble(x2_key);
  std::string file_path =
      folder_path + "state_" + std::to_string(iter_id) + ".txt";
  std::ofstream file;
  file.open(file_path);
  file << x1 << " " << x2 << "\n";
  file << state.lambda << "\n";
  file << state.error << "\n";
  file << num_trials << "\n";
  file.close();
}

void SaveTrial(const IELMTrial &trial, int iter_id, int trial_id,
               const std::string &folder_path) {
  Values new_values;
  for (const auto &[key, manifold] : trial.nonlinear_update.new_manifolds) {
    new_values.insert(manifold.values());
  }

  double new_x1 = new_values.atDouble(x1_key);
  double new_x2 = new_values.atDouble(x2_key);

  double delta_x1 = trial.linear_update.tangent_vector.at(x1_key)(0);
  double delta_x2 = trial.linear_update.tangent_vector.at(x2_key)(0);

  std::string file_path = folder_path + "trial_" + std::to_string(iter_id) +
                          "_" + std::to_string(trial_id) + ".txt";
  std::ofstream file;
  file.open(file_path);
  file << delta_x1 << " " << delta_x2 << "\n";
  file << new_x1 << " " << new_x2 << "\n";
  file << trial.linear_update.lambda << "\n";
  file << trial.nonlinear_update.new_error << "\n";
  file << trial.step_is_successful << "\n";
  file.close();
}

void SaveDetails(const IELMItersDetails &iters_details,
                 const std::string &folder_path) {
  std::filesystem::create_directory(folder_path);
  // int total_trials = 0;
  for (int iter_id = 0; iter_id < iters_details.size(); iter_id++) {
    const auto &iter_detail = iters_details.at(iter_id);
    const auto &state = iter_detail.state;
    SaveState(state, iter_id, folder_path, iter_detail.trials.size());
    // total_trials += iter_detail.trials.size();
    for (int trial_id = 0; trial_id < iter_detail.trials.size(); trial_id++) {
      const auto &trial = iter_detail.trials.at(trial_id);
      SaveTrial(trial, iter_id, trial_id, folder_path);
    }
  }
}

/** Optimization using Type1 manifold optimizer. */
void OptimizeSO2() {

  Point2 goal_point(-2.0, 0.5);
  Point2 init_point(-0.6, -0.8);
  Vector2 cost_sigmas(10.0, 1.0);

  NonlinearFactorGraph graph;
  graph.addPrior(x1_key, goal_point.x(),
                 noiseModel::Isotropic::Sigma(1, cost_sigmas(0)));
  graph.addPrior(x2_key, goal_point.y(),
                 noiseModel::Isotropic::Sigma(1, cost_sigmas(1)));

  EqualityConstraints constraints;
  Double_ x1(x1_key);
  Double_ x2(x2_key);
  Double_ dist_square(dist_square_func, x1, x2);
  Double_ dist_error = dist_square - Double_(1.0);
  double tolerance = 1e-3;
  constraints.emplace_shared<gtdynamics::DoubleExpressionEquality>(dist_error,
                                                                   tolerance);

  InequalityConstraints i_constraints;

  Values init_values;
  init_values.insert(x1_key, init_point.x());
  init_values.insert(x2_key, init_point.y());

  IELMParams ielm_params;
  ielm_params.lm_params.setVerbosityLM("SUMMARY");
  // ielm_params.lm_params.setMaxIterations(1);
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_creator = std::make_shared<MatrixBasisCreator>();
  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.minModelFidelity = 0.5;

  //// Optimization with fixed sigmas
  {
    BarrierRetractor::Params barrier_params(lm_params, 0.1);
    barrier_params.init_values_as_x = false;
    iecm_params->retractor_creator =
        std::make_shared<BarrierRetractorCreator>(barrier_params);
    IELMOptimizer optimizer(ielm_params, iecm_params);
    auto result =
        optimizer.optimize(graph, constraints, i_constraints, init_values);
    // result.print();
    std::cout << "error: " << graph.error(result) << "\n";
    const auto &details = optimizer.details();
    std::string folder_path = "../../data/normal_proj_so2/";
    SaveSummary(init_point, goal_point, cost_sigmas, details.size(),
                folder_path);
    SaveDetails(details, folder_path);
  }

  //// Optimization with varying sigmas
  {
    BarrierRetractor::Params barrier_params =
        BarrierRetractor::Params::VarySigmas(lm_params);
    barrier_params.init_values_as_x = false;
    iecm_params->retractor_creator =
        std::make_shared<BarrierRetractorCreator>(barrier_params);
    IELMOptimizer optimizer(ielm_params, iecm_params);
    auto result =
        optimizer.optimize(graph, constraints, i_constraints, init_values);
    // result.print();
    std::cout << "error: " << graph.error(result) << "\n";
    const auto &details = optimizer.details();
    std::string folder_path = "../../data/special_proj_so2/";
    SaveSummary(init_point, goal_point, cost_sigmas, details.size(),
                folder_path);
    SaveDetails(details, folder_path);
  }
}

int main() {
  OptimizeSO2();
  return 0;
}
