/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file EqualityOnlyBaseline.h
 * @brief Controlled comparison with the existing equality-only CM optimizer.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/cmcopt/IECMPoint.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/cmopt/BasisRetractor.h>
#include <gtdynamics/cmopt/LMManifoldOptimizer.h>

namespace g1_example {

// Configure equality projection to preserve the same free coordinates.
inline gtdynamics::RetractionParams::shared_ptr equalityBaselineParams() {
  auto params = std::make_shared<gtdynamics::RetractionParams>();
  params->feasibleThreshold = 1e-14;
  params->lmParams.absoluteErrorTol = 1e-16;
  params->lmParams.relativeErrorTol = 1e-12;
  return params;
}

inline gtdynamics::ManifoldOptimizationProblem manifoldLmProblem(
    const gtsam::NonlinearFactorGraph& cost, const gtdynamics::IECMPoints& points,
    const gtsam::Values& unconstrained,
    const gtdynamics::TangentSpaceBasisCreator::shared_ptr& basisCreator,
    const gtdynamics::BasisKeyFunction& select) {
  using namespace gtdynamics;
  ManifoldOptimizationProblem problem;
  problem.values = unconstrained;
  problem.unconstrainedKeys = gtsam::KeySet(unconstrained.keys());
  std::map<gtsam::Key, gtsam::Key> ownership;
  for (const auto& [key, point] : points) {
    if (point.manifold()->inequalityCount(point) != 0)
      throw std::invalid_argument("Manifold LM comparison requires equality-only points");
    auto params = std::make_shared<EConstraintManifold::Params>();
    params->equalityBasisCreator = basisCreator;
    params->retractorCreator = std::make_shared<BasisRetractorCreator>(
        select, equalityBaselineParams());
    params->affineEqualities = point.hasAffineEqualities();
    const auto type = std::make_shared<EConstraintManifold>(
        *point.eConstraints(), point.values(), *params);
    const auto value = type->point(point.values());
    problem.types.push_back(type);
    if (value.dim()) {
      problem.values.insert(key, value);
      problem.manifoldKeys.insert(key);
    } else {
      problem.fixedPoints.emplace(key, value);
    }
    for (gtsam::Key ambient : point.values().keys()) ownership[ambient] = key;
  }
  problem.graph =
      ManifoldOptimizer::manifoldGraph(cost, ownership, problem.fixedPoints);
  return problem;
}

}  // namespace g1_example
