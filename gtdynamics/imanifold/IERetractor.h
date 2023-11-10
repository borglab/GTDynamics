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
    bool use_varying_sigma = false;
    std::shared_ptr<VectorValues> metric_sigmas = NULL;
    bool init_values_as_x = true; // 
    bool check_feasible = true;
    double feasible_threshold = 1e-3;

    Params() = default;

    /** Constructor using the same sigma. */
    Params(const LevenbergMarquardtParams &_lm_params,
           const double &_prior_sigma)
        : lm_params(_lm_params), prior_sigma(_prior_sigma),
          use_varying_sigma(false), metric_sigmas(NULL) {}

    /** Constructor using varying sigmas. */
    Params(const LevenbergMarquardtParams &_lm_params,
           const std::shared_ptr<VectorValues> &_metric_sigmas)
        : lm_params(_lm_params), prior_sigma(0.0), use_varying_sigma(true),
          metric_sigmas(_metric_sigmas) {}

    static Params VarySigmas(const LevenbergMarquardtParams &_lm_param =
                                 LevenbergMarquardtParams()) {
      return Params(_lm_param, std::make_shared<VectorValues>());
    }

    template <typename CONTAINER>
    void addPriors(const Values &values, const CONTAINER &keys,
                   NonlinearFactorGraph &graph) const {
      if (use_varying_sigma) {
        AddGeneralPriors(values, keys, *metric_sigmas, graph);
      } else {
        AddGeneralPriors(values, keys, prior_sigma, graph);
      }
    }
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
  using Params = BarrierRetractor::Params;

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
};

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Base class of retractor creator. */
class IERetractorCreator {
public:
  bool use_varying_sigmas = false;
  std::shared_ptr<VectorValues> metric_sigmas;

public:
  using shared_ptr = std::shared_ptr<IERetractorCreator>;

  IERetractorCreator() : use_varying_sigmas(false), metric_sigmas(NULL) {}

  IERetractorCreator(const std::shared_ptr<VectorValues> &_metric_sigmas)
      : use_varying_sigmas(true), metric_sigmas(_metric_sigmas) {}

  IERetractorCreator(bool _use_varying_sigmas,
                     const std::shared_ptr<VectorValues> &_metric_sigmas)
      : use_varying_sigmas(_use_varying_sigmas), metric_sigmas(_metric_sigmas) {
  }

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
      : IERetractorCreator(params.use_varying_sigma, params.metric_sigmas),
        params_(params),
        retractor_(std::make_shared<BarrierRetractor>(params_)) {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override {
    return retractor_;
  }

  virtual ~BarrierRetractorCreator() {}
};

} // namespace gtsam
