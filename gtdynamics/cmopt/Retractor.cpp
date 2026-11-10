/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Retractor.cpp
 * @brief Equality-manifold retraction interfaces and common behavior.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/cmcopt/IECMPoint.h>
#include <gtdynamics/cmopt/EConstraintManifold.h>

#include <iostream>

namespace gtdynamics {
using namespace gtsam;

IECMPoint Retractor::retract(
    const IECMPoint *point, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices,
    IERetractionInfo *retract_info) const {
  if (!point) throw std::invalid_argument("Missing equality retraction point");
  if (blocking_indices && !blocking_indices->empty())
    throw std::invalid_argument("Equality retraction has no boundary blocks");

  // Point factories supply an isolated workspace for these mutable kernels.
  auto &workspace = const_cast<Retractor &>(*this);
  Values values = workspace.retract(point->values(), delta);
  return point->manifold()->point(point->bindings(), values);
}

void RetractorCreator::validatePointType(const IECMPoint &point) {
  if (!std::dynamic_pointer_cast<const EConstraintManifold>(point.manifold()))
    throw std::invalid_argument(
        "Equality retraction policies require an EConstraintManifold");
}

RetractionParams::shared_ptr RetractorCreator::boundParams(
    const IECMPoint &point) const {
  auto result = std::make_shared<RetractionParams>(*equalityParams_);
  if (result->lmParams.ordering) {
    gtsam::Ordering ordering;
    for (Key slot : *result->lmParams.ordering) {
      const auto found = point.bindings().find(slot);
      if (found != point.bindings().end()) ordering.push_back(found->second);
    }
    result->lmParams.ordering = ordering;
  }
  return result;
}

IERetractorCreator::shared_ptr RetractorCreator::cloneFor(
    const KeySet &keys) const {
  auto result = std::dynamic_pointer_cast<RetractorCreator>(clone());
  result->params_ = IERetractorParams::snapshot(params_, &keys);
  result->scopeOrdering(keys);
  return result;
}

IERetractor::shared_ptr RetractorCreator::create(const IECMPoint &point) const {
  validatePointType(point);
  auto constraints =
      std::make_shared<NonlinearEqualityConstraints>(*point.eConstraints());
  auto bound = std::dynamic_pointer_cast<RetractorCreator>(clone());
  bound->equalityParams_ = boundParams(point);
  return bound->create(constraints);
}

void Retractor::setOptimizerGraph(
    MutableLMOptimizer &optimizer, const NonlinearFactorGraph &graph) const {
  const auto params = equalityParams_->lmParamsFor(graph.keys());
  if (params.ordering) optimizer.setGraph(graph, *params.ordering);
  else optimizer.setGraph(graph);
}

void Retractor::checkFeasible(const NonlinearFactorGraph &graph,
                              const Values &values) const {
  if (equalityParams_->checkFeasible) {
    if (graph.error(values) > equalityParams_->feasibleThreshold) {
      std::cout << "fail: " << graph.error(values) << "\n";
    }
  }
}


}  // namespace gtdynamics
