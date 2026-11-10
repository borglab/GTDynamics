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
#include <gtdynamics/cmcopt/IECMPoint.h>
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
    if (!basis_keys)
      throw std::invalid_argument("Basis-key projection requires coordinate keys");
    basis_keys_ = *basis_keys;
  }
}

ProjectionRetractor::ProjectionRetractor(const ProjectionRetractor &other)
    : Retractor(other), merit_graph_(other.merit_graph_),
      basis_keys_(other.basis_keys_) {}

IERetractor::shared_ptr ProjectionRetractorCreator::create(
    const IECMPoint &point) const {
  validatePointType(point);
  const auto settings = boundParams(point);
  auto constraints =
      std::make_shared<NonlinearEqualityConstraints>(*point.eConstraints());
  if (!settings->useBasisKeys)
    return std::make_shared<ProjectionRetractor>(constraints, settings);
  const auto keys = point.eBasis()->basisKeys();
  if (!keys)
    throw std::invalid_argument(
        "Basis-key projection requires a coordinate-key equality basis");
  return std::make_shared<ProjectionRetractor>(constraints, settings, *keys);
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
  if (equalityParams_->useBasisKeys) {
    AddGeneralPriors(values_retract_base, basis_keys_, equalityParams_->sigma, graph);
  } else {
    AddGeneralPriors(values_retract_base, equalityParams_->sigma, graph);
  }
  // const Values &init_values =
  //     equalityParams_->applyBaseRetraction ? values_retract_base : values;
  const Values &init_values = values_retract_base;
  gtsam::LevenbergMarquardtOptimizer optimizer_with_priors(
      graph, init_values, equalityParams_->lmParamsFor(graph.keys()));
  const Values &result = optimizer_with_priors.optimize();
  if (equalityParams_->useBasisKeys &&
      optimizer_with_priors.error() < equalityParams_->feasibleThreshold) {
    return result;
  }

  // optimize without priors
  gtsam::LevenbergMarquardtOptimizer optimizer_without_priors(
      merit_graph_, result, equalityParams_->lmParamsFor(merit_graph_.keys()));
  const Values &final_result = optimizer_without_priors.optimize();
  checkFeasible(merit_graph_, final_result);
  return final_result;
}

} // namespace gtdynamics
