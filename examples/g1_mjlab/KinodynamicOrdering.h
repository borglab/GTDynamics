/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file KinodynamicOrdering.h
 * @brief Eliminate trajectory components and shared variables in time order.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/inference/Ordering.h>

#include <algorithm>
#include <map>

namespace g1_example {
/** Eliminate each slice's components before retiring shared variables at their
 * last observation. Stance landmarks leave after their final observing slice;
 * the IMU bias remains until its last factor. Unobserved free variables leave
 * at firstSlice. Component keys encode the trajectory slice in DynamicsSymbol.
 */
inline gtsam::Ordering chronologicalOrdering(
    const gtsam::NonlinearFactorGraph& cost,
    const gtdynamics::IEManifoldValues& manifolds,
    const gtsam::Values& unconstrained, size_t firstSlice) {
  const auto ownership =
      gtdynamics::IEOptimizer::varToManifoldKeyMap(manifolds);
  std::map<gtsam::Key, size_t> lastSlice;
  for (gtsam::Key key : unconstrained.keys()) lastSlice[key] = firstSlice;
  for (const auto& factor : cost) {
    size_t last = firstSlice;
    for (gtsam::Key key : factor->keys()) {
      const auto owner = ownership.find(key);
      if (owner != ownership.end())
        last = std::max(last, gtdynamics::DynamicsSymbol(owner->second).time());
    }
    for (gtsam::Key key : factor->keys())
      if (unconstrained.exists(key))
        lastSlice[key] = std::max(lastSlice[key], last);
  }
  std::map<size_t, gtsam::KeyVector> blocks;
  for (const auto& [key, manifold] : manifolds)
    blocks[gtdynamics::DynamicsSymbol(key).time()].push_back(key);
  for (const auto& [key, last] : lastSlice) blocks[last].push_back(key);
  gtsam::Ordering ordering;
  for (const auto& [slice, keys] : blocks)
    ordering.insert(ordering.end(), keys.begin(), keys.end());
  return ordering;
}
}  // namespace g1_example
