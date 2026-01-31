/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptimizer.cpp
 * @brief Base class constrained optimization.
 * @author Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>

namespace gtsam {
/* ************************************************************************* */
std::shared_ptr<LevenbergMarquardtOptimizer>
ConstrainedOptimizer::CreateLMOptimizer(
    const NonlinearFactorGraph &graph, const Values &values,
    const bool store_lm_details,
    const LevenbergMarquardtParams &lm_params) const {
  if (store_lm_details) {
    return std::make_shared<HistoryLMOptimizer>(graph, values, lm_params);
  } else {
    return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                         lm_params);
  }
}

/* ************************************************************************* */
std::pair<std::vector<Values>, std::vector<size_t>>
ConstrainedOptimizer::RetrieveLMItersValues(
    std::shared_ptr<LevenbergMarquardtOptimizer> optimizer) const {
  auto history_optimizer =
      std::static_pointer_cast<HistoryLMOptimizer>(optimizer);
  std::vector<Values> lm_iters_values;
  std::vector<size_t> lm_inner_iters;
  size_t prev_accum_inner_iters = 0;

  for (const auto &state : history_optimizer->states()) {
    if (state.totalNumberInnerIterations == 0) {
      continue;
    }
    lm_iters_values.push_back(state.values);
    lm_inner_iters.push_back(state.totalNumberInnerIterations -
                             prev_accum_inner_iters);
    prev_accum_inner_iters = state.totalNumberInnerIterations;
  }
  return std::make_pair(lm_iters_values, lm_inner_iters);
}

} // namespace gtsam
