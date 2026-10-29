/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file BasisRetractor.cpp
 * @brief BasisRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/BasisRetractor.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "factors/ConstVarFactor.h"
#include "utils/DynamicsSymbol.h"
#include "utils/values.h"

namespace gtdynamics {
using namespace gtsam;

BasisRetractor::BasisRetractor(
    const NonlinearEqualityConstraints::shared_ptr &constraints,
    const RetractionParams::shared_ptr &params, const KeyVector &basis_keys)
    : Retractor(params), merit_graph_(constraints->penaltyGraph()),
      basis_keys_(basis_keys), optimizer_(params->lmParams) {
  NonlinearFactorGraph graph;
  KeySet fixed_keys(basis_keys.begin(), basis_keys.end());
  for (const auto &factor : merit_graph_) {
    // check if the factor contains fixed keys
    bool contain_fixed_keys = false;
    for (const Key &key : factor->keys()) {
      if (fixed_keys.exists(key)) {
        contain_fixed_keys = true;
        break;
      }
    }
    // update the factor if it constains fixed keys
    if (contain_fixed_keys) {
      NoiseModelFactor::shared_ptr noise_factor =
          std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto const_var_factor =
          std::make_shared<ConstVarFactor>(noise_factor, fixed_keys);
      if (const_var_factor->checkActive()) {
        factors_with_fixed_vars_.push_back(const_var_factor);
        graph.add(const_var_factor);
      }
    } else {
      graph.add(factor);
    }
  }
  // set the graph for optimizer
  optimizer_.setGraph(graph);
}

/* ************************************************************************* */
Values BasisRetractor::retractConstraints(const Values &values) {
  static const bool debug_retractor_stats = []() {
    const char *env = std::getenv("GTDYN_DEBUG_RETRACTOR_STATS");
    return env && std::string(env) != "0";
  }();
  static size_t total_calls = 0;
  static size_t fast_path_calls = 0;
  static double total_ms = 0.0;

  const auto start = std::chrono::steady_clock::now();
  ++total_calls;

  // set fixed values for the const variable factors
  for (auto &factor : factors_with_fixed_vars_) {
    factor->setFixedValues(values);
  }
  // set initial values for optimizer
  Values opt_values;
  for (const Key &key : optimizer_.graph().keys()) {
    opt_values.insert(key, values.at(key));
  }

  // Fast path: if constraints are already satisfied to threshold, avoid
  // repeatedly running tiny LM solves.
  const double initial_error = optimizer_.graph().error(opt_values);
  if (initial_error <= params_->feasibleThreshold) {
    ++fast_path_calls;
    Values result = opt_values;
    for (const Key &key : basis_keys_) {
      result.insert(key, values.at(key));
    }
    const auto end = std::chrono::steady_clock::now();
    total_ms +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() *
        1e-3;
    if (debug_retractor_stats && total_calls % 200 == 0) {
      std::cout << "[RETRACTOR] calls=" << total_calls
                << ", fast_path=" << fast_path_calls
                << ", avg_ms=" << (total_ms / static_cast<double>(total_calls))
                << ", last_initial_error=" << initial_error << "\n";
    }
    return result;
  }

  optimizer_.setValues(std::move(opt_values));
  // optimize
  Values result = optimizer_.optimize();

  if (params_->recompute) {
    if (optimizer_.graph().error(result) > 1e-5) {
      std::cout << "recompute\n";
      for (const Key &key : basis_keys_) {
        result.insert(key, values.at(key));
      }
      auto optimizer = gtsam::LevenbergMarquardtOptimizer(merit_graph_, result);
      result = optimizer.optimize();
      checkFeasible(merit_graph_, result);
      return result;
    }
  }

  // add fixed variables to the result
  for (const Key &key : basis_keys_) {
    result.insert(key, values.at(key));
  }
  const auto end = std::chrono::steady_clock::now();
  total_ms += std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                  .count() *
              1e-3;
  if (debug_retractor_stats && total_calls % 200 == 0) {
    std::cout << "[RETRACTOR] calls=" << total_calls
              << ", fast_path=" << fast_path_calls
              << ", avg_ms=" << (total_ms / static_cast<double>(total_calls))
              << ", last_initial_error=" << initial_error << "\n";
  }
  return result;
}

} // namespace gtdynamics
