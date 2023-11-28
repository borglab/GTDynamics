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

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/utils/GraphUtils.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IERetractor::shared_ptr Vision60HierarchicalRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.values().keys());
    return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_,
                                                              basis_keys);
  }
  return std::make_shared<KinodynamicHierarchicalRetractor>(manifold, params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60BarrierRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (use_basis_keys_) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.values().keys());
    return std::make_shared<BarrierRetractor>(params_, basis_keys);
  }
  return std::make_shared<BarrierRetractor>(params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60MultiPhaseHierarchicalRetractorCreator::create(
    const IEConstraintManifold &manifold) const {
  if (manifold.values().size() == 1) {
    return std::make_shared<BarrierRetractor>(params_);
  }
  if (use_basis_keys_) {
    size_t k =
        gtdynamics::DynamicsSymbol(*manifold.values().keys().begin()).time();
    KeyVector basis_keys =
        vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(manifold.values().keys());
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
        vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(manifold.values().keys());
    return std::make_shared<BarrierRetractor>(params_, basis_keys);
  }
  return std::make_shared<BarrierRetractor>(params_);
}

/* ************************************************************************* */
TspaceBasis::shared_ptr Vision60MultiPhaseTspaceBasisCreator::create(
    const EqualityConstraints::shared_ptr constraints, const Values &values) const {
  if (values.size() == 1) {
    return std::make_shared<OrthonormalBasis>(constraints, values, params_);
  }
  size_t k = gtdynamics::DynamicsSymbol(*values.keys().begin()).time();
  KeyVector basis_keys =
      vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(values.keys());
  return std::make_shared<EliminationBasis>(constraints, values, params_, basis_keys);
}

} // namespace gtsam
