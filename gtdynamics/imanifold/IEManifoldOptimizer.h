/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEGradientDescentOptimizer.h
 * @brief First order optimization on manifolds with boundaries/corners.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {
typedef std::map<Key, IEConstraintManifold> IEManifoldValues;
typedef std::map<Key, ConstraintManifold> EManifoldValues;

class IEOptimizer {
public:
  IEOptimizer() {}

  void print() const { std::cout << "Hello\n"; }

  virtual Values optimize(
      const NonlinearFactorGraph &graph,
      const gtdynamics::EqualityConstraints &e_constraints,
      const gtdynamics::InequalityConstraints &i_constraints,
      const gtsam::Values &initial_values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params,
      gtdynamics::ConstrainedOptResult *intermediate_result = nullptr) const {
    auto manifolds = IdentifyManifolds(e_constraints, i_constraints,
                                       initial_values, iecm_params);
    return optimizeManifolds(graph, manifolds, intermediate_result);
  }

  virtual Values optimizeManifolds(const NonlinearFactorGraph &graph,
                                   const IEManifoldValues &manifolds,
                                   gtdynamics::ConstrainedOptResult *
                                       intermediate_result = nullptr) const = 0;

public:
  static std::map<Key, Key>
  Var2ManifoldKeyMap(const IEManifoldValues &manifolds);

  static IEManifoldValues IdentifyManifolds(
      const gtdynamics::EqualityConstraints &e_constraints,
      const gtdynamics::InequalityConstraints &i_constraints,
      const gtsam::Values &values,
      const IEConstraintManifold::Params::shared_ptr &iecm_params);

  static Values CollectManifoldValues(const IEManifoldValues &manifolds);

  static VectorValues ComputeTangentVector(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  /// Note: xi is given with the e_basis that only consider equality
  /// constraints.
  static std::pair<IndexSetMap, VectorValues>
  ProjectTangentCone(const IEManifoldValues &manifolds, const VectorValues &v);

  static IEManifoldValues RetractManifolds(const IEManifoldValues &manifolds,
                                           const VectorValues &delta);

  static Values EManifolds(const IEManifoldValues &manifolds);

  static std::pair<Values, Values>
  EManifolds(const IEManifoldValues &manifolds,
             const IndexSetMap &active_indices);

  static bool IsSameMode(const IEManifoldValues &manifolds1,
                         const IEManifoldValues &manifolds2);

  static IndexSetMap
  IdentifyChangeIndices(const IEManifoldValues &manifolds,
                        const IEManifoldValues &new_manifolds);

  static IndexSetMap
  IdentifyApproachingIndices(const IEManifoldValues &manifolds,
                             const IEManifoldValues &new_manifolds,
                             const IndexSetMap &change_indices_map);

  static IEManifoldValues
  MoveToBoundaries(const IEManifoldValues &manifolds,
                   const IndexSetMap &approach_indices_map);

};

} // namespace gtsam
