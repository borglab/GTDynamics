/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionRetractor.cpp
 * @brief ProjectionRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/ProjectionRetractor.h>
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

ProjectionRetractor::ProjectionRetractor(
    const NonlinearEqualityConstraints::shared_ptr &constraints,
    const RetractionParams::shared_ptr &params,
    std::optional<const KeyVector> basis_keys)
    : Retractor(params), merit_graph_(constraints->penaltyGraph()) {
  if (params->useBasisKeys) {
    basis_keys_ = *basis_keys;
  }
}

/* ************************************************************************* */
Values ProjectionRetractor::retractConstraints(const Values &values) {
  gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph_, values);
  return optimizer.optimize();
}

/* ************************************************************************* */
Values ProjectionRetractor::retract(const Values &values,
                                    const VectorValues &delta) {
  // optimize with priors
  NonlinearFactorGraph graph = merit_graph_;
  Values values_retract_base = retractBaseVariables(values, delta);
  if (params_->useBasisKeys) {
    AddGeneralPriors(values_retract_base, basis_keys_, params_->sigma, graph);
  } else {
    AddGeneralPriors(values_retract_base, params_->sigma, graph);
  }
  // const Values &init_values =
  //     params_->applyBaseRetraction ? values_retract_base : values;
  const Values &init_values = values_retract_base;
  gtsam::LevenbergMarquardtOptimizer optimizer_with_priors(graph, init_values,
                                                           params_->lmParams);
  const Values &result = optimizer_with_priors.optimize();
  if (params_->useBasisKeys &&
      optimizer_with_priors.error() < params_->feasibleThreshold) {
    return result;
  }

  // optimize without priors
  gtsam::LevenbergMarquardtOptimizer optimizer_without_priors(
      merit_graph_, result, params_->lmParams);
  const Values &final_result = optimizer_without_priors.optimize();
  checkFeasible(merit_graph_, final_result);
  return final_result;
}

} // namespace gtdynamics
