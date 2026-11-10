/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CMRetractor.h
 * @brief Shared retraction policies, metrics, and isolated workspaces.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace gtdynamics {
class CMPoint;
class ConstVarFactor;

// Common projection settings contain no boundary or inequality solver policy.
struct CMRetractorParams {
  using shared_ptr = std::shared_ptr<CMRetractorParams>;
  gtsam::LevenbergMarquardtParams lmParams;
  double priorSigma = 1.0;
  bool useVaryingSigma = false;
  bool scaleVaryingSigma = false;
  std::shared_ptr<gtsam::VectorValues> metricSigmas;
  bool initValuesAsX = true;
  bool checkFeasible = true;
  double feasibleThreshold = 1e-3;
  bool ensureFeasible = false;

  /// Snapshot mutable metric entries, optionally within one component.
  void snapshotMetric(const CMRetractorParams& source,
                      const gtsam::KeySet* scope = nullptr);
  static shared_ptr snapshot(const shared_ptr& source,
                             const gtsam::KeySet* scope = nullptr);

  /// Preserve repeated-prior and varying-sigma weighting conventions.
  gtsam::VectorValues priorPrecisions(const gtsam::Values& values,
                                      const gtsam::KeyVector& keys) const;

  template <typename CONTAINER>
  void addPriors(const gtsam::Values& values, const CONTAINER& keys,
                 gtsam::NonlinearFactorGraph& graph) const {
    const auto precisions =
        priorPrecisions(values, gtsam::KeyVector(keys.begin(), keys.end()));
    for (const auto& [key, weights] : precisions)
      graph.emplace_shared<GeneralPriorFactor>(
          key, values.at(key),
          gtsam::noiseModel::Diagonal::Precisions(weights));
  }
};

/// Numerical projection failure; optimizers can reject and retry the trial.
/// Invalid inputs and programming errors retain their usual exception types.
class RetractionFailure : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Solver-work counters are independent of the point's constraint category.
struct CMRetractionInfo {
  size_t numLMIterations = 0;
  size_t numALSolveCalls = 0;
  size_t numPenaltySolveCalls = 0;
  size_t numFeasibleTrials = 0;
  size_t numFeasibleProjections = 0;
};

/// A prepared policy maps ambient increments back to ambient point values.
class CMRetractor {
 public:
  using shared_ptr = std::shared_ptr<CMRetractor>;
  virtual ~CMRetractor() = default;

  /// Every call owns its mutable settings and solver workspace.
  virtual shared_ptr cloneWorkspace() const = 0;
  virtual shared_ptr bindPoint(const CMPoint& point) const;
  virtual gtsam::Values retractValues(
      const CMPoint& point, const gtsam::VectorValues& delta,
      CMRetractionInfo* info = nullptr) const = 0;
  virtual const CMRetractorParams& commonParams() const = 0;

  void scopeMetric(const gtsam::KeySet& keys);
  void setMetricSigmas(const gtsam::VectorValues& sigmas);

  /// A copied factor graph must reference the copied fixed-variable factors.
  static void detachFixedFactors(
      gtsam::NonlinearFactorGraph& graph,
      std::vector<std::shared_ptr<ConstVarFactor>>& factors);

 protected:
  virtual CMRetractorParams& mutableCommonParams() = 0;
};

/// Runtime types snapshot a policy; points receive independent preparation.
class CMRetractorCreator {
 public:
  using shared_ptr = std::shared_ptr<CMRetractorCreator>;
  virtual ~CMRetractorCreator() = default;
  virtual shared_ptr clonePolicy() const = 0;
  virtual shared_ptr cloneForKeys(const gtsam::KeySet& keys) const = 0;
  virtual CMRetractor::shared_ptr prepare(const CMPoint& point) const = 0;
  virtual const CMRetractorParams& commonParams() const = 0;
};
}  // namespace gtdynamics
