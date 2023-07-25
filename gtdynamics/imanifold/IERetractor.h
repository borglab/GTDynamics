/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IERetractor.h
 * @brief Retractor for IEConstraintManifold.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

class IEConstraintManifold;

// enum IERetractType { Barrier = 0, HalfSphere = 1, CP=2 };

/** Base class that implements the retraction operation for the constraint
 * manifold. */
class IERetractor {

public:
  using shared_ptr = std::shared_ptr<IERetractor>;

  /// Default constructor.
  IERetractor() {}

  virtual ~IERetractor() {}

  /// Retract the base variables that compose the constraint manifold.
  virtual IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const = 0;

  virtual IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices) const;
};

class HalfSphereRetractor : public IERetractor {
public:
  HalfSphereRetractor() : IERetractor() {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;
};

/** Retraction by performing barrier optimization. */
class BarrierRetractor : public IERetractor {
public:
  BarrierRetractor() : IERetractor() {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;
};

class CartPoleWithFrictionRetractor : public IERetractor {
protected:
  const IECartPoleWithFriction &cp_;

public:
  CartPoleWithFrictionRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;

  IEConstraintManifold retract1(const IEConstraintManifold *manifold,
                                const VectorValues &delta) const;
};

class CPBarrierRetractor : public IERetractor {
protected:
  const IECartPoleWithFriction &cp_;

public:
  CPBarrierRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;
};

} // namespace gtsam
