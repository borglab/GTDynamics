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
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

class IEConstraintManifold;

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

/** Retraction by performing barrier optimization. */
class BarrierRetractor : public IERetractor {
public:
  struct Params {
    LevenbergMarquardtParams lm_params = LevenbergMarquardtParams();
    double prior_sigma = 1.0;

    Params() = default;
  };

protected:
  const Params &params_;
  bool use_basis_keys_;
  KeyVector basis_keys_;

public:
  BarrierRetractor(const Params &params,
                   const std::optional<KeyVector> &basis_keys = {})
      : IERetractor(), params_(params) {
    if (basis_keys) {
      use_basis_keys_ = true;
      basis_keys_ = *basis_keys;
    } else {
      use_basis_keys_ = false;
    }
  }

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;
};

/**
 * Retractor that hierarchically solves the retraction probelm. It first solves
 * q-level by only considering constraints involving poses and joint angles.
 * Then, it solves for link twists and joint velocities using the v-level
 * constraints and the known q-level values. Finally, it solves for all a-level
 * and dynamics level variables as remainig ad-level factors are linear given
 * the qv-level variables.
 */
class KinodynamicHierarchicalRetractor : public IERetractor {

public:
  struct Params {
    LevenbergMarquardtParams lm_params;
    double prior_sigma;
    bool check_feasible;
    double feasible_threshold;
  };

protected:
  const Params &params_;
  KeyVector basis_keys_;

  NonlinearFactorGraph merit_graph_;
  NonlinearFactorGraph graph_q_, graph_v_, graph_ad_;
  IndexSet i_indices_q_, i_indices_v_, i_indices_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

public:
  KinodynamicHierarchicalRetractor(
      const IEConstraintManifold &manifold, const Params &params,
      const std::optional<KeyVector> &basis_keys = {});

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const override;

protected:
  void checkFeasible(const NonlinearFactorGraph &graph,
                     const Values &values) const;
};

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Base class of retractor creator. */
class IERetractorCreator {

public:
  using shared_ptr = std::shared_ptr<IERetractorCreator>;
  IERetractorCreator() {}

  virtual IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const = 0;
};

/** Creates the same retractor for all manifolds */
class UniversalIERetractorCreator : public IERetractorCreator {
protected:
  IERetractor::shared_ptr retractor_;

public:
  UniversalIERetractorCreator(const IERetractor::shared_ptr &retractor)
      : IERetractorCreator(), retractor_(retractor) {}

  virtual ~UniversalIERetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override {
    return retractor_;
  }
};

class BarrierRetractorCreator : public IERetractorCreator {
protected:
  BarrierRetractor::Params params_;
  IERetractor::shared_ptr retractor_;

public:
  BarrierRetractorCreator(
      const BarrierRetractor::Params &params = BarrierRetractor::Params())
      : IERetractorCreator(), params_(params),
        retractor_(std::make_shared<BarrierRetractor>(params_)) {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override {
    return retractor_;
  }

  virtual ~BarrierRetractorCreator() {}
};

} // namespace gtsam
