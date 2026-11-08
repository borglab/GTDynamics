/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file main_shared_ieconstraint_manifold.cpp
 * @brief Couple two differently bound points of one manually defined type.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

#include "ManualIEConstraintManifold.h"

int main() {
  using namespace manual_ieconstraint_manifold;
  const auto type = lineType();

  // Two ambient bindings share the same runtime manifold definition.
  CMPoints points;
  points.emplace(Symbol('c', 0),
                 type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 0)));
  points.emplace(Symbol('c', 1),
                 type->point({{0, 20}, {1, 21}}, lineValues(20, 21, 0)));

  // Couple the two points so they participate in one global QP.
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(10, Vector(Vector1(3)),
                                            noiseModel::Unit::Create(1));
  graph.emplace_shared<PriorFactor<Vector>>(20, Vector(Vector1(-3)),
                                            noiseModel::Unit::Create(1));
  graph.emplace_shared<BetweenFactor<Vector>>(10, 20, Vector(Vector1(0)),
                                              noiseModel::Unit::Create(1));

  auto discovery = std::make_shared<IEConstraintManifold::Params>();
  IELMParams params;
  params.lmParams.maxIterations = 10;
  IELMOptimizer optimizer(params, discovery);
  const auto manual = optimizer.optimize(graph, points, {});

  // Rediscover the same constrained problem from its ambient factor views.
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  for (const auto& [key, point] : points) {
    equalities.add(*point.eConstraints());
    inequalities.add(*point.iConstraints());
  }

  const auto automatic = optimizer.optimize(graph, equalities, inequalities,
                                            ambientValues(points));

  manual.print("Shared type, distinct ambient bindings");
  std::cout << "Manual/automatic objectives: " << graph.error(manual) << " / "
            << graph.error(automatic) << '\n';
  return manual.equals(automatic, 1e-5) ? 0 : 1;
}
