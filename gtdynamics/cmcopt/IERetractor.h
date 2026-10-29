/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IERetractor.h
 * @brief Retractor for IEConstraintManifold.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {
using namespace gtsam;

class IEConstraintManifold;

struct IERetractorParams {
  using shared_ptr = std::shared_ptr<IERetractorParams>;
  LevenbergMarquardtParams lmParams = LevenbergMarquardtParams();
  double priorSigma = 1.0;
  bool useVaryingSigma = false;
  bool scaleVaryingSigma = false;
  std::shared_ptr<VectorValues> metricSigmas = NULL;
  /// Opt into affine reduction in BarrierRetractor; nonlinear manifolds stay ambient.
  bool useReducedCoordinates = false;
  /// Coordinate preconditioning only; false retains the original basis metric.
  bool whitenAffineCoordinates = true;
  bool initValuesAsX = true; //
  bool checkFeasible = true;
  double feasibleThreshold = 1e-3;
  bool ensureFeasible = false;
  PenaltyOptimizerParams::shared_ptr penaltyParams =
      std::make_shared<PenaltyOptimizerParams>();

  IERetractorParams() = default;

  /** Constructor using the same sigma. */
  IERetractorParams(const LevenbergMarquardtParams &_lmParams,
                    const double &_priorSigma)
      : lmParams(_lmParams), priorSigma(_priorSigma),
        useVaryingSigma(false), metricSigmas(NULL) {
    penaltyParams->lmParams = lmParams;
  }

  /** Constructor using varying sigmas. */
  IERetractorParams(const LevenbergMarquardtParams &_lmParams,
                    const std::shared_ptr<VectorValues> &_metric_sigmas)
      : lmParams(_lmParams), priorSigma(0.0), useVaryingSigma(true),
        metricSigmas(_metric_sigmas) {
    penaltyParams->lmParams = lmParams;
  }

  static IERetractorParams::shared_ptr createVaryingSigmas(
      const LevenbergMarquardtParams &_lmParam = LevenbergMarquardtParams()) {
    return std::make_shared<IERetractorParams>(
        _lmParam, std::make_shared<VectorValues>());
  }

  /** Effective diagonal weights, shared by ambient and reduced projection.
   * Missing varying-sigma keys are omitted; repeated keys accumulate weight.
   */
  VectorValues priorPrecisions(const Values &values, const KeyVector &keys) const;

  /// Add the quadratic prior term that pulls the retraction solution toward
  /// the trial point x + delta.
  template <typename CONTAINER>
  void addPriors(const Values &values, const CONTAINER &keys,
                 NonlinearFactorGraph &graph) const {
    const auto precisions = priorPrecisions(values, KeyVector(keys.begin(), keys.end()));
    for (const auto &[key, weights] : precisions)
      graph.emplace_shared<GeneralPriorFactor>(
          key, values.at(key), noiseModel::Diagonal::Precisions(weights));
  }
};

struct IERetractionInfo {
  size_t numLMIterations = 0;
  size_t numALSolveCalls = 0;
  size_t numPenaltySolveCalls = 0;
  size_t numFeasibleTrials = 0;
  size_t numFeasibleProjections = 0;
};

/// Base class for the feasibility-restoring map R_x(delta) used after each
/// cone-constrained step.
class IERetractor {

protected:
  IERetractorParams::shared_ptr params_;

public:
  using shared_ptr = std::shared_ptr<IERetractor>;

  /// Default constructor.
  IERetractor(const IERetractorParams::shared_ptr &params =
                  std::make_shared<IERetractorParams>())
      : params_(params) {}

  virtual ~IERetractor() {}

  /// Retract the ambient variables back to h(x)=0, g(x)<=0, optionally with
  /// selected blocking inequalities promoted to equalities.
  virtual IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const = 0;

  /// Zero-step retraction that lands directly on the requested boundary face.
  virtual IEConstraintManifold
  moveToBoundary(const IEConstraintManifold *manifold,
                 const IndexSet &blocking_indices,
                 IERetractionInfo *retract_info = nullptr) const;

  const IERetractorParams::shared_ptr &params() const { return params_; }
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

} // namespace gtdynamics
