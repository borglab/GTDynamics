/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TspaceBasis.h
 * @brief Basis for tangent space of constraint manifold. Detailed definition of
 * tangent space and basis are available at Boumal20book Sec.8.4.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/cmopt/TspaceBasis.h>
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

using EqualityConstraints = gtsam::NonlinearEqualityConstraints;
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
struct RetractParams {
 public:
  using shared_ptr = std::shared_ptr<RetractParams>;

  // Member variables
  bool check_feasible = false;
  double feasible_threshold = 1e-5;
  LevenbergMarquardtParams lm_params;
  bool use_basis_keys = false;
  double sigma = 1.0;
  bool apply_base_retraction = false;
  bool recompute = false;

  // Constructor
  RetractParams() = default;
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
  RetractParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor(const RetractParams::shared_ptr &params =
                std::make_shared<RetractParams>())
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
 * Retractor using unconstrained optimization on the penalty graph.
 *
 * This corresponds to approximate metric projection behavior used in the
 * tutorial.
 *
 * @see README.md#retraction
 */
class UoptRetractor : public Retractor {
 protected:
  MutableLMOptimizer optimizer_;

 public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   */
  UoptRetractor(const EqualityConstraints::shared_ptr &constraints,
                const RetractParams::shared_ptr &params =
                    std::make_shared<RetractParams>());

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

  /// Retraction Inplace
  Values retractConstraints(Values &&values) override;
};

/**
 * Retractor implementing projection-style updates with optional priors.
 *
 * @see README.md#retraction
 */
class ProjRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;

 public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Optional basis keys used when `use_basis_keys` is true.
   */
  ProjRetractor(const EqualityConstraints::shared_ptr &constraints,
                const RetractParams::shared_ptr &params,
                std::optional<const KeyVector> basis_keys = {});

  /**
   * Retract with projection-style retraction.
   * @param values Base values composing the constraint manifold.
   * @param delta Tangent update.
   * @return Retraction result projected toward constraint manifold.
   */
  Values retract(const Values &values, const VectorValues &delta) override;

  Values retractConstraints(const Values &values) override;
};

/**
 * Retractor that fixes basis variables and solves for the remaining variables.
 *
 * This matches the basis-variable retraction idea in the tutorial and paper.
 *
 * @see README.md#retraction
 * @see README.md#tangent-basis
 */
class BasisRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;
  MutableLMOptimizer optimizer_;
  std::vector<std::shared_ptr<ConstVarFactor>> factors_with_fixed_vars_;

 public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Basis keys held fixed during inner solve.
   */
  BasisRetractor(const EqualityConstraints::shared_ptr &constraints,
                 const RetractParams::shared_ptr &params,
                 const KeyVector &basis_keys);

  virtual ~BasisRetractor() {}

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

 protected:
  /**
   * Build reduced retraction graph with basis keys treated as fixed.
   * @param constraints Equality constraints for the component.
   * @param basis_keys Basis keys held fixed during inner solve.
   */
  void constructGraph(const EqualityConstraints::shared_ptr &constraints,
                      const KeyVector &basis_keys);
};

/**
 * Customized hierarchical retractor for kinodynamics components.
 *
 * Variables/factors are split into levels (q, v, and acceleration/dynamics)
 * and solved in sequence with level-specific optimizers and priors.
 *
 * @see README.md#retraction
 */
class DynamicsRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  MutableLMOptimizer optimizer_wp_q_, optimizer_wp_v_, optimizer_wp_ad_;
  MutableLMOptimizer optimizer_np_q_, optimizer_np_v_, optimizer_np_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

 public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Optional basis keys.
   */
  DynamicsRetractor(const EqualityConstraints::shared_ptr &constraints,
                    const RetractParams::shared_ptr &params,
                    std::optional<const KeyVector> basis_keys = {});

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

 protected:
  /**
   * Classify keys by dynamics level.
   * @param keys Input key container.
   * @param q_keys Output position-level keys.
   * @param v_keys Output velocity-level keys.
   * @param ad_keys Output acceleration/dynamics-level keys.
   */
  template <typename CONTAINER>
  static void classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                           KeySet &v_keys, KeySet &ad_keys);

  /**
   * Refresh linear priors in a graph for selected keys.
   * @param values Values used as prior means.
   * @param keys Keys whose priors are updated.
   * @param graph Graph whose trailing prior factors are replaced.
   */
  void updatePriors(const Values &values, const KeySet &keys,
                    NonlinearFactorGraph &graph);
};

/**
 * Factory interface for creating retractor instances.
 *
 * @see README.md#retraction
 */
class RetractorCreator {
 protected:
  RetractParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<RetractorCreator>;
  RetractorCreator(RetractParams::shared_ptr params) : params_(params) {}

  RetractParams::shared_ptr params() { return params_; }

  virtual ~RetractorCreator() {}

  virtual Retractor::shared_ptr create(
      const EqualityConstraints::shared_ptr constraints) const = 0;
};

/**
 * Factory for `UoptRetractor`.
 *
 * @see README.md#retraction
 */
class UoptRetractorCreator : public RetractorCreator {
 public:
  UoptRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  virtual ~UoptRetractorCreator() {}

  Retractor::shared_ptr create(
      const EqualityConstraints::shared_ptr constraints) const override {
    return std::make_shared<UoptRetractor>(constraints, params_);
  }
};

/**
 * Factory for `ProjRetractor`.
 *
 * @see README.md#retraction
 */
class ProjRetractorCreator : public RetractorCreator {
 public:
  ProjRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  virtual ~ProjRetractorCreator() {}

  Retractor::shared_ptr create(
      const EqualityConstraints::shared_ptr constraints) const override {
    return std::make_shared<ProjRetractor>(constraints, params_);
  }
};

/**
 * Factory for `BasisRetractor`.
 *
 * @see README.md#retraction
 * @see README.md#tangent-basis
 */
class BasisRetractorCreator : public RetractorCreator {
 public:
  BasisKeyFunc basis_key_func_;

 public:
  BasisRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  /**
   * Constructor with custom basis-key selector.
   * @param basis_key_func Callback selecting basis keys.
   * @param params Retraction parameters.
   */
  BasisRetractorCreator(
      BasisKeyFunc basis_key_func,
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~BasisRetractorCreator() {}

  Retractor::shared_ptr create(
      const EqualityConstraints::shared_ptr constraints) const override {
    KeyVector basis_keys = basis_key_func_(constraints->keyVector());
    return std::make_shared<BasisRetractor>(constraints, params_, basis_keys);
  }
};

/**
 * Factory for `DynamicsRetractor`.
 *
 * @see README.md#retraction
 */
class DynamicsRetractorCreator : public RetractorCreator {
 protected:
  BasisKeyFunc basis_key_func_;

 public:
  DynamicsRetractorCreator(RetractParams::shared_ptr params)
      : RetractorCreator(params) {}

  /**
   * Constructor with custom basis-key selector.
   * @param params Retraction parameters.
   * @param basis_key_func Callback selecting basis keys.
   */
  DynamicsRetractorCreator(RetractParams::shared_ptr params,
                           BasisKeyFunc basis_key_func)
      : RetractorCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~DynamicsRetractorCreator() {}

  Retractor::shared_ptr create(
      const EqualityConstraints::shared_ptr constraints) const override {
    if (params_->use_basis_keys) {
      KeyVector basis_keys = basis_key_func_(constraints->keyVector());
      return std::make_shared<DynamicsRetractor>(constraints, params_,
                                                 basis_keys);
    } else {
      return std::make_shared<DynamicsRetractor>(constraints, params_);
    }
  }
};

}  // namespace gtdynamics
