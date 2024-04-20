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
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/constrained_optimizer/PenaltyOptimizer.h>
#include <gtdynamics/constraints/InequalityConstraint.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

class IEConstraintManifold;

struct IERetractorParams {
  using shared_ptr = std::shared_ptr<IERetractorParams>;
  LevenbergMarquardtParams lm_params = LevenbergMarquardtParams();
  double prior_sigma = 1.0;
  bool use_varying_sigma = false;
  bool scale_varying_sigma = false;
  std::shared_ptr<VectorValues> metric_sigmas = NULL;
  bool init_values_as_x = true; //
  bool check_feasible = true;
  double feasible_threshold = 1e-3;
  bool ensure_feasible = false;
  gtsam::PenaltyParameters::shared_ptr penalty_params =
      std::make_shared<gtsam::PenaltyParameters>();

  IERetractorParams() = default;

  /** Constructor using the same sigma. */
  IERetractorParams(const LevenbergMarquardtParams &_lm_params,
                    const double &_prior_sigma)
      : lm_params(_lm_params), prior_sigma(_prior_sigma),
        use_varying_sigma(false), metric_sigmas(NULL) {}

  /** Constructor using varying sigmas. */
  IERetractorParams(const LevenbergMarquardtParams &_lm_params,
                    const std::shared_ptr<VectorValues> &_metric_sigmas)
      : lm_params(_lm_params), prior_sigma(0.0), use_varying_sigma(true),
        metric_sigmas(_metric_sigmas) {}

  static IERetractorParams::shared_ptr VarySigmas(
      const LevenbergMarquardtParams &_lm_param = LevenbergMarquardtParams()) {
    return std::make_shared<IERetractorParams>(
        _lm_param, std::make_shared<VectorValues>());
  }

  template <typename CONTAINER>
  void addPriors(const Values &values, const CONTAINER &keys,
                 NonlinearFactorGraph &graph) const {
    if (use_varying_sigma) {
      if (scale_varying_sigma) {
        AddGeneralPriors(values, keys, *metric_sigmas, graph, prior_sigma);
      } else {
        AddGeneralPriors(values, keys, *metric_sigmas, graph);
      }
    } else {
      AddGeneralPriors(values, keys, prior_sigma, graph);
    }
  }
};

struct IERetractInfo {
  size_t num_lm_iters = 0;
};

/** Base class that implements the retraction operation for the constraint
 * manifold. */
class IERetractor {

protected:
  IERetractorParams::shared_ptr params_;
  gtsam::PenaltyOptimizer penalty_optimizer_;

public:
  using shared_ptr = std::shared_ptr<IERetractor>;

  /// Default constructor.
  IERetractor(const IERetractorParams::shared_ptr &params =
                  std::make_shared<IERetractorParams>())
      : params_(params), penalty_optimizer_(params_->penalty_params) {}

  virtual ~IERetractor() {}

  /// Retract the base variables that compose the constraint manifold.
  virtual IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo *retract_info = nullptr) const = 0;

  virtual IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices,
                 IERetractInfo *retract_info = nullptr) const;

  const IERetractorParams::shared_ptr &params() const { return params_; }
};

/** Retraction by performing penalty optimization. */
class BarrierRetractor : public IERetractor {

protected:
  bool use_basis_keys_;
  KeyVector basis_keys_;

public:
  BarrierRetractor(const IERetractorParams::shared_ptr &params,
                   const std::optional<KeyVector> &basis_keys = {})
      : IERetractor(params) {
    if (basis_keys) {
      use_basis_keys_ = true;
      basis_keys_ = *basis_keys;
    } else {
      use_basis_keys_ = false;
    }
  }

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo *retract_info = nullptr) const override;

  IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices,
                 IERetractInfo *retract_info = nullptr) const override;
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
  using Params = IERetractorParams;

protected:
  KeyVector basis_keys_;
  NonlinearFactorGraph merit_graph_;
  NonlinearFactorGraph graph_q_, graph_v_, graph_ad_;
  IndexSet i_indices_q_, i_indices_v_, i_indices_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

public:
  KinodynamicHierarchicalRetractor(
      const IEConstraintManifold &manifold, const Params::shared_ptr &params,
      const std::optional<KeyVector> &basis_keys = {});

  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractInfo *retract_info = nullptr) const override;
};

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Base class of retractor creator. */
class IERetractorCreator {
protected:
  IERetractorParams::shared_ptr params_;

public:
  using shared_ptr = std::shared_ptr<IERetractorCreator>;

  IERetractorCreator(const IERetractorParams::shared_ptr &params =
                         std::make_shared<IERetractorParams>())
      : params_(params) {}

  virtual IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const = 0;

  const IERetractorParams::shared_ptr &params() const { return params_; }
};

/** Creates the same retractor for all manifolds */
class UniversalIERetractorCreator : public IERetractorCreator {
protected:
  IERetractor::shared_ptr retractor_;

public:
  UniversalIERetractorCreator(const IERetractor::shared_ptr &retractor)
      : IERetractorCreator(retractor->params()), retractor_(retractor) {}

  virtual ~UniversalIERetractorCreator() {}

  IERetractor::shared_ptr
  create(const IEConstraintManifold &manifold) const override {
    return retractor_;
  }
};

class BarrierRetractorCreator : public UniversalIERetractorCreator {

public:
  BarrierRetractorCreator(IERetractorParams::shared_ptr params =
                              std::make_shared<IERetractorParams>())
      : UniversalIERetractorCreator(
            std::make_shared<BarrierRetractor>(params)) {}

  virtual ~BarrierRetractorCreator() {}
};

} // namespace gtsam
