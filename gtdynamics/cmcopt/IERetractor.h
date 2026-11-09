/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IERetractor.h
 * @brief Retractor for CMPoint.
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

#include <stdexcept>

namespace gtdynamics {
using namespace gtsam;

class CMPoint;

struct IERetractorParams {
  using shared_ptr = std::shared_ptr<IERetractorParams>;
  LevenbergMarquardtParams lmParams = LevenbergMarquardtParams();
  double priorSigma = 1.0;
  bool useVaryingSigma = false;
  bool scaleVaryingSigma = false;
  std::shared_ptr<VectorValues> metricSigmas = NULL;
  /// Opt into affine reduction in BarrierRetractor; nonlinear points stay
  /// ambient.
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

  /// Deep snapshot of mutable metric and penalty settings.
  static shared_ptr snapshot(const shared_ptr &source,
                             const KeySet *scope = nullptr) {
    if (!source) throw std::invalid_argument("Missing retraction settings");
    auto result = std::make_shared<IERetractorParams>(*source);
    if (source->metricSigmas) {
      if (!scope)
        result->metricSigmas =
            std::make_shared<VectorValues>(*source->metricSigmas);
      else {
        result->metricSigmas = std::make_shared<VectorValues>();
        for (Key key : *scope)
          if (source->metricSigmas->exists(key))
            result->metricSigmas->insert(key, source->metricSigmas->at(key));
      }
    }
    if (source->penaltyParams)
      result->penaltyParams =
          std::make_shared<PenaltyOptimizerParams>(*source->penaltyParams);
    return result;
  }

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

/** A numerically unsuccessful feasibility projection. LM may reject this trial
 * and retry with stronger damping. Invalid inputs and programming errors use
 * their usual exception types and must still propagate to the caller.
 */
class RetractionFailure : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
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
      : params_(IERetractorParams::snapshot(params)) {}

  /// Copies create isolated mutable settings, never shared projection state.
  IERetractor(const IERetractor &other)
      : params_(IERetractorParams::snapshot(other.params_)) {}

  virtual ~IERetractor() {}
  /// Detach the mutable fixed-variable factors in a copied workspace.
  static void detachFixedFactors(
      NonlinearFactorGraph &graph,
      std::vector<std::shared_ptr<ConstVarFactor>> &factors) {
    std::map<const ConstVarFactor *, std::shared_ptr<ConstVarFactor>> copies;
    for (auto &factor : factors) {
      auto copy = std::make_shared<ConstVarFactor>(*factor);
      copies.emplace(factor.get(), copy);
      factor = copy;
    }
    for (auto &factor : graph) {
      const auto fixed = dynamic_cast<const ConstVarFactor *>(factor.get());
      if (fixed && copies.count(fixed)) factor = copies.at(fixed);
    }
  }
  /// Clone a prototype into one retraction workspace.
  virtual shared_ptr clone() const = 0;
  /// Bind local policy keys and metric entries to one point's ambient keys.
  virtual shared_ptr bind(const CMPoint &point) const;
  /// Restrict a prepared policy snapshot to the component's ambient variables.
  void scopeMetric(const KeySet &keys) {
    params_ = IERetractorParams::snapshot(params_, &keys);
  }
  /// Trial-local metric update; only isolated workspaces receive it.
  void setMetricSigmas(const VectorValues &sigmas) {
    params_->metricSigmas = std::make_shared<VectorValues>(sigmas);
  }

  /// Retract the ambient variables back to h(x)=0, g(x)<=0, optionally with
  /// selected blocking inequalities promoted to equalities.
  virtual CMPoint retract(
      const CMPoint *point, const VectorValues &delta,
      const std::optional<IndexSet> &blocking_indices = {},
      IERetractionInfo *retract_info = nullptr) const = 0;

  /// Zero-step retraction that lands directly on the requested boundary face.
  virtual CMPoint moveToBoundary(
      const CMPoint *point, const IndexSet &blocking_indices,
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
      : params_(IERetractorParams::snapshot(params)) {}

  /// Snapshot the policy configuration before a runtime type captures it.
  virtual shared_ptr clone() const = 0;
  /// Capture only component-local metric entries from a trajectory policy.
  virtual shared_ptr cloneFor(const KeySet &keys) const {
    auto result = clone();
    result->params_ = IERetractorParams::snapshot(params_, &keys);
    return result;
  }
  /// Deep-copy the common creator settings.
  IERetractorCreator(const IERetractorCreator &other)
      : params_(IERetractorParams::snapshot(other.params_)) {}

  virtual IERetractor::shared_ptr
  create(const CMPoint &point) const = 0;

  const IERetractorParams::shared_ptr &params() const { return params_; }
};

/** Creates the same retractor for all points */
class UniversalIERetractorCreator : public IERetractorCreator {
protected:
  IERetractor::shared_ptr retractor_;

public:
  UniversalIERetractorCreator(const IERetractor::shared_ptr &retractor)
      : IERetractorCreator(retractor->params()), retractor_(retractor) {}

  UniversalIERetractorCreator(const UniversalIERetractorCreator &other)
      : IERetractorCreator(other), retractor_(other.retractor_->clone()) {}
  virtual ~UniversalIERetractorCreator() {}
  shared_ptr clone() const override {
    return std::make_shared<UniversalIERetractorCreator>(*this);
  }
  shared_ptr cloneFor(const KeySet &keys) const override {
    auto result = std::make_shared<UniversalIERetractorCreator>(*this);
    result->params_ = IERetractorParams::snapshot(params_, &keys);
    result->retractor_->scopeMetric(keys);
    return result;
  }

  IERetractor::shared_ptr
  create(const CMPoint &point) const override;
};

} // namespace gtdynamics
