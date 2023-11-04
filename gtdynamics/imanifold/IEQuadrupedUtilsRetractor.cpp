/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include "imanifold/IERetractor.h"
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtdynamics/manifold/GeneralPriorFactor.h>
#include <gtdynamics/utils/DebugUtils.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IERetractor::shared_ptr Vision60HierarchicalRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.eCC());
    return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_,
                                                              basis_keys);
  }
  return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60BarrierRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.eCC());
    return std::make_shared<BarrierRetractor>(params_, basis_keys);
  }
  return std::make_shared<BarrierRetractor>(params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60MultiPhaseHierarchicalRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    size_t k =
        gtdynamics::DynamicsSymbol(*manifold.values().keys().begin()).time();
    KeyVector basis_keys =
        vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(manifold.eCC());
    return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_,
                                                              basis_keys);
  }
  return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60MultiPhaseBarrierRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    size_t k =
        gtdynamics::DynamicsSymbol(*manifold.values().keys().begin()).time();
    KeyVector basis_keys =
        vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(manifold.eCC());
    return std::make_shared<BarrierRetractor>(params_, basis_keys);
  }
  return std::make_shared<BarrierRetractor>(params_);
}

/* ************************************************************************* */
TspaceBasis::shared_ptr Vision60MultiPhaseTspaceBasisCreator::create(
    const ConnectedComponent::shared_ptr cc, const Values &values) const {
  size_t k = gtdynamics::DynamicsSymbol(*values.keys().begin()).time();
  KeyVector basis_keys =
      vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(cc);
  size_t manifold_dim = values.dim() - cc->constraints_.dim();
  return TspaceBasis::create(params_, cc, values, basis_keys, manifold_dim);
}

} // namespace gtsam
