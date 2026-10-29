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

#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/cmopt/TangentSpaceBasis.h>
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
class Retractor {
 protected:
  RetractionParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor(const RetractionParams::shared_ptr &params =
                std::make_shared<RetractionParams>())
      : params_(params) {}

  virtual ~Retractor() {}

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
class RetractorCreator {
 protected:
  RetractionParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<RetractorCreator>;
  RetractorCreator(RetractionParams::shared_ptr params) : params_(params) {}

  RetractionParams::shared_ptr params() { return params_; }

  virtual ~RetractorCreator() {}

  virtual Retractor::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints) const = 0;
};

}  // namespace gtdynamics
