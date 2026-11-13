/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IERetractor.h
 * @brief Retractor for IECMPoint.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/constrained/CMRetractor.h>
#include <gtdynamics/constrained/MultiJacobian.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <stdexcept>

namespace gtdynamics {
using namespace gtsam;

class IECMPoint;

struct IERetractorParams : CMRetractorParams {
  using shared_ptr = std::shared_ptr<IERetractorParams>;
  /// Opt into affine reduction in BarrierRetractor; nonlinear points stay
  /// ambient.
  bool useReducedCoordinates = false;
  /// Coordinate preconditioning only; false retains the original basis metric.
  bool whitenAffineCoordinates = true;
  PenaltyOptimizerParams::shared_ptr penaltyParams =
      std::make_shared<PenaltyOptimizerParams>();

  IERetractorParams() = default;

  /// Deep snapshot of mutable metric and penalty settings.
  static shared_ptr snapshot(const shared_ptr &source,
                             const KeySet *scope = nullptr) {
    if (!source) throw std::invalid_argument("Missing retraction settings");
    auto result = std::make_shared<IERetractorParams>(*source);
    result->snapshotMetric(*source, scope);
    if (source->penaltyParams)
      result->penaltyParams =
          std::make_shared<PenaltyOptimizerParams>(*source->penaltyParams);
    return result;
  }

  /** Constructor using the same sigma. */
  IERetractorParams(const LevenbergMarquardtParams &_lmParams,
                    const double &_priorSigma) {
    lmParams = _lmParams;
    priorSigma = _priorSigma;
    penaltyParams->lmParams = lmParams;
  }

  /** Constructor using varying sigmas. */
  IERetractorParams(const LevenbergMarquardtParams &_lmParams,
                    const std::shared_ptr<VectorValues> &_metric_sigmas) {
    lmParams = _lmParams;
    priorSigma = 0.0;
    useVaryingSigma = true;
    metricSigmas = _metric_sigmas;
    penaltyParams->lmParams = lmParams;
  }

  static IERetractorParams::shared_ptr createVaryingSigmas(
      const LevenbergMarquardtParams &_lmParam = LevenbergMarquardtParams()) {
    return std::make_shared<IERetractorParams>(
        _lmParam, std::make_shared<VectorValues>());
  }
};

// Retain the typed IE API while sharing solver-work diagnostics.
using IERetractionInfo = CMRetractionInfo;

/// Base class for the feasibility-restoring map R_x(delta) used after each
/// cone-constrained step.
class IERetractor : public CMRetractor {

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
  /// Clone a prototype into one retraction workspace.
  virtual shared_ptr clone() const = 0;
  /// Bind local policy keys and metric entries to one point's ambient keys.
  virtual shared_ptr bind(const IECMPoint &point) const;
  /// Common point operations delegate to the typed IE policy.
  CMRetractor::shared_ptr cloneWorkspace() const override { return clone(); }
  CMRetractor::shared_ptr bindPoint(const CMPoint &point) const override;
  Values retractValues(const CMPoint &point, const VectorValues &delta,
                       CMRetractionInfo *info = nullptr) const override;
  const CMRetractorParams &commonParams() const override { return *params_; }

  /// Retract the ambient variables back to h(x)=0, g(x)<=0, optionally with
  /// selected blocking inequalities promoted to equalities.
  virtual IECMPoint retract(
      const IECMPoint *point, const VectorValues &delta,
      const std::optional<IndexSet> &blocking_indices = {},
      IERetractionInfo *retract_info = nullptr) const = 0;

  /// Zero-step retraction that lands directly on the requested boundary face.
  virtual IECMPoint moveToBoundary(
      const IECMPoint *point, const IndexSet &blocking_indices,
      IERetractionInfo *retract_info = nullptr) const;

  const IERetractorParams::shared_ptr &params() const { return params_; }

protected:
  CMRetractorParams &mutableCommonParams() override { return *params_; }
};

/* ************************************************************************* */
/* <=================== Factory class for Retractor =======================> */
/* ************************************************************************* */

/** Base class of retractor creator. */
class IERetractorCreator : public CMRetractorCreator {
protected:
  IERetractorParams::shared_ptr params_;

public:
  using shared_ptr = std::shared_ptr<IERetractorCreator>;
  virtual ~IERetractorCreator() = default;

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
  create(const IECMPoint &point) const = 0;

  CMRetractorCreator::shared_ptr clonePolicy() const override {
    return clone();
  }
  CMRetractorCreator::shared_ptr cloneForKeys(const KeySet &keys) const override {
    return cloneFor(keys);
  }
  CMRetractor::shared_ptr prepare(const CMPoint &point) const override;
  const CMRetractorParams &commonParams() const override { return *params_; }

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
  create(const IECMPoint &point) const override;
};

} // namespace gtdynamics
