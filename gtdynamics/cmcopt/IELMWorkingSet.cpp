/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IELMWorkingSet.cpp
 * @brief Coordinate component-local working sets for the coupled LM objective.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/IELMWorkingSet.h>

namespace gtdynamics {
namespace internal {
WorkingSetResult refineWorkingSet(const GaussianFactorGraph& cost,
                                  const ComponentLinearizations& components,
                                  const VectorValues& initial,
                                  const IndexSetMap& blockingIndices,
                                  const NonlinearOptimizerParams& params,
                                  size_t maxIterations) {
  using Row = IEManifoldLinearization::Row;
  std::map<Key, IEManifoldLinearization::RowSet> selected;
  for (const auto& [key, local] : components) {
    if (local->indices().empty()) continue;
    const auto found = blockingIndices.find(key);
    selected[key] = local->initialRows(
        found == blockingIndices.end() ? IndexSet{} : found->second,
        initial.at(key));
  }
  WorkingSetResult result{initial, {}, 0, false};
  for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
    WorkingFaces charts;
    for (const auto& [key, rows] : selected)
      if (!rows.empty())
        charts.emplace(key, components.at(key)->workingFace(rows));
    const VectorValues candidate = solveWorkingFaces(cost, charts, params);
    ++result.numSolves;
    const VectorValues direction = candidate - result.delta;
    // The coupled gradient determines the dual equations inside each component.
    if (candidate.equals(result.delta, 1e-7)) {
      const VectorValues gradient = cost.gradient(candidate);
      double mostNegative = -1e-7;
      std::optional<std::pair<Key, Row>> leaving;
      for (const auto& [key, rows] : selected) {
        const auto local =
            components.at(key)->leavingRow(gradient.at(key), rows);
        if (local.second && local.first < mostNegative) {
          mostNegative = local.first;
          leaving = {key, *local.second};
        }
      }
      result.delta = candidate;
      if (!leaving) {
        result.converged = true;
        break;
      }
      selected.at(leaving->first).erase(leaving->second);
      continue;
    }
    // Preserve key/row order and strict tie-breaking when choosing a boundary.
    double step = 1.0;
    std::optional<std::pair<Key, Row>> entering;
    for (const auto& [key, rows] : selected) {
      const auto local = components.at(key)->boundaryStep(
          result.delta.at(key), direction.at(key), rows, step);
      if (local.second) {
        step = local.first;
        entering = {key, *local.second};
      }
    }
    result.delta += step * direction;
    if (entering) selected.at(entering->first).insert(entering->second);
  }
  for (const auto& [key, rows] : selected) {
    const auto& local = *components.at(key);
    const auto complete = local.completeBlocks(rows);
    if (!complete.empty()) result.blockingIndices.emplace(key, complete);
    if (!local.violated(result.delta.at(key), 1e-7).empty())
      result.converged = false;
  }
  return result;
}
}  // namespace internal
}  // namespace gtdynamics
