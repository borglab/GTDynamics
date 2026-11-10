/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Retractor.h
 * @brief Equality-manifold retraction interfaces and common behavior.
 * @author: Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/constrained/MultiJacobian.h>
#include <gtdynamics/constrained/TangentSpaceBasis.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <optional>
#include <vector>

namespace gtdynamics {

using gtsam::NonlinearEqualityConstraints;
using gtsam::Key;
using gtsam::KeySet;
using gtsam::KeyVector;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using gtsam::VectorValues;

/**
 * Parameters for constraint-manifold retraction.
 *
 * These parameters control feasibility checks, solver behavior, optional basis
 * key usage, and prior strengths used by different retractor implementations.
 *
 * @see README.md#retraction
 */
struct RetractionParams {
 public:
  using shared_ptr = std::shared_ptr<RetractionParams>;

  // Member variables
  bool checkFeasible = false;
  double feasibleThreshold = 1e-5;
  LevenbergMarquardtParams lmParams;
  bool useBasisKeys = false;
  double sigma = 1.0;
  bool applyBaseRetraction = false;
  bool recompute = false;

  // Constructor
  RetractionParams() = default;

  /// Keep explicit ordering within one component or inner solve graph.
  LevenbergMarquardtParams lmParamsFor(const KeySet &keys) const {
    auto result = lmParams;
    if (result.ordering) {
      gtsam::Ordering ordering;
      for (Key key : *result.ordering)
        if (keys.count(key)) ordering.push_back(key);
      result.ordering = ordering;
    }
    return result;
  }
};

/**
 * Base class for constraint-manifold retraction.
 *
 * Retraction is implemented as:
 * 1. Retract base variables in ambient space.
 * 2. Enforce equality constraints to return to the constraint manifold.
 *
 * Concrete subclasses define different feasibility-enforcement strategies.
 *
 * @see README.md#retraction
 */
class Retractor : public IERetractor {
 protected:
  RetractionParams::shared_ptr equalityParams_;

 public:
  using shared_ptr = std::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor(const RetractionParams::shared_ptr &params =
                std::make_shared<RetractionParams>())
      : IERetractor(),
        equalityParams_(std::make_shared<RetractionParams>(*params)) {}

  /// Copies retain policy settings while concrete copies detach solver work.
  Retractor(const Retractor &other)
      : IERetractor(other),
        equalityParams_(
            std::make_shared<RetractionParams>(*other.equalityParams_)) {}

  virtual ~Retractor() {}

  /// Keep equality tolerance semantics separate from corner projection settings.
  const RetractionParams::shared_ptr &equalityParams() const {
    return equalityParams_;
  }

  /// Shared point wrapper around the existing ambient retraction kernels.
  IECMPoint retract(const IECMPoint *point, const VectorValues &delta,
                    const std::optional<IndexSet> &blocking_indices = {},
                    IERetractionInfo *retract_info = nullptr) const override;

  /**
   * Retract base variables in ambient space before enforcing constraints.
   * @param values Base values composing the constraint manifold.
   * @param delta Tangent update in ambient coordinates.
   * @return Updated base values prior to constraint projection.
   */
  virtual Values retractBaseVariables(const Values &values,
                                      const VectorValues &delta) const {
    return values.retract(delta);
  }

  /** Retraction operation
   * @param values base values composing the constraint manifold
   * @param delta tangent vector
   * @return retracted point on the manifold
   */
  virtual Values retract(Values &&values, const VectorValues &delta) {
    return retractConstraints(retractBaseVariables(values, delta));
  }

  /** Retraction operation
   * @param values base values composing the constraint manifold
   * @param delta tangent vector
   * @return retracted point on the manifold
   */
  virtual Values retract(const Values &values, const VectorValues &delta) {
    return retractConstraints(retractBaseVariables(values, delta));
  }

  /**
   * Retract values to satisfy constraints.
   * @param values Candidate values, potentially infeasible.
   * @return Constraint-satisfying values.
   */
  virtual Values retractConstraints(Values &&values) {
    return retractConstraints((const Values &)values);
  };

  /** Given values of variables in CCC that may violate the constraints, compute
   * the values that satisfy the constraints. */
  virtual Values retractConstraints(const Values &values) = 0;

 protected:
  /// Each inner graph uses the applicable part of the chosen ordering.
  void setOptimizerGraph(MutableLMOptimizer &optimizer,
                         const NonlinearFactorGraph &graph) const;

  /**
   * Optionally report infeasibility.
   * @param graph Constraint graph used for feasibility check.
   * @param values Values to verify.
   */
  void checkFeasible(const NonlinearFactorGraph &graph,
                     const Values &values) const;
};

/**
 * Factory interface for creating retractor instances.
 *
 * @see README.md#retraction
 */
class RetractorCreator : public IERetractorCreator {
 protected:
  RetractionParams::shared_ptr equalityParams_;

  /// Validate the equality-only policy contract once when preparing a point.
  static void validatePointType(const IECMPoint &point);

  /// Map local policy keys without changing the frozen type configuration.
  RetractionParams::shared_ptr boundParams(const IECMPoint &point) const;
  void scopeOrdering(const KeySet &keys) {
    equalityParams_->lmParams = equalityParams_->lmParamsFor(keys);
  }

 public:
  using shared_ptr = std::shared_ptr<RetractorCreator>;
  RetractorCreator(RetractionParams::shared_ptr params)
      : IERetractorCreator(), equalityParams_(params) {}

  RetractorCreator(const RetractorCreator &other)
      : IERetractorCreator(other),
        equalityParams_(
            std::make_shared<RetractionParams>(*other.equalityParams_)) {}

  RetractionParams::shared_ptr params() { return equalityParams_; }
  const RetractionParams::shared_ptr &equalityParams() const {
    return equalityParams_;
  }

  IERetractorCreator::shared_ptr cloneFor(const KeySet &keys) const override;

  /// Prepare from bound views; no generic rekeying of embedded expressions.
  IERetractor::shared_ptr create(const IECMPoint &point) const override;

  virtual ~RetractorCreator() {}

  virtual Retractor::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints) const = 0;
};

}  // namespace gtdynamics
