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

#include <gtdynamics/utils/values.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtdynamics/constraints/EqualityConstraint.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using gtsam::EqualityConstraints;

namespace gtsam {

/// Parameters for constraint manifold retraction operation.
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

/** Base class that implements the retraction operation for the constraint
 * manifold. */
class Retractor {
 protected:
  // ConnectedComponent::shared_ptr cc_;
  RetractParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor(const RetractParams::shared_ptr &params =
                std::make_shared<RetractParams>())
      : params_(params) {}

  virtual ~Retractor() {}

  /// Retract the base variables that compose the constraint manifold.
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

  /** Given values of variables in CCC that may violate the constraints, compute
   * the values that satisfy the constraints. */
  virtual Values retractConstraints(Values &&values) {
    return retractConstraints((const Values &)values);
  };

  /** Given values of variables in CCC that may violate the constraints, compute 
   * the values that satisfy the constraints. */
  virtual Values retractConstraints(const Values &values) = 0;

 protected:
  void checkFeasible(const NonlinearFactorGraph &graph,
                     const Values &values) const;
};

/** Retractor with unconstrained optimization. */
class UoptRetractor : public Retractor {
 protected:
  MutableLMOptimizer optimizer_;

 public:
  /// Constructor.
  UoptRetractor(const EqualityConstraints::shared_ptr &constraints,
                const RetractParams::shared_ptr &params =
                    std::make_shared<RetractParams>());

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

  /// Retraction Inplace
  Values retractConstraints(Values &&values) override;
};

/** Retractor with metric projection. */
class ProjRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;

 public:
  /// Constructor.
  ProjRetractor(const EqualityConstraints::shared_ptr &constraints,
                const RetractParams::shared_ptr &params,
                std::optional<const KeyVector> basis_keys = {});

  /// Retraction operation.
  Values retract(const Values &values, const VectorValues &delta) override;

  Values retractConstraints(const Values &values) override;
};

/** Retractor by specifying the basis variables. */
class BasisRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;
  MutableLMOptimizer optimizer_;
  std::vector<std::shared_ptr<ConstVarFactor>> factors_with_fixed_vars_;

 public:
  /// Constructor.
  BasisRetractor(const EqualityConstraints::shared_ptr &constraints,
                 const RetractParams::shared_ptr &params,
                 const KeyVector &basis_keys);

  virtual ~BasisRetractor() {}

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

 protected:
  void constructGraph(const EqualityConstraints::shared_ptr &constraints,
                      const KeyVector &basis_keys);
};

/** Customized retractor for kinodynamics manifold. */
class DynamicsRetractor : public Retractor {
 protected:
  NonlinearFactorGraph merit_graph_;
  MutableLMOptimizer optimizer_wp_q_, optimizer_wp_v_, optimizer_wp_ad_;
  MutableLMOptimizer optimizer_np_q_, optimizer_np_v_, optimizer_np_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

 public:
  /// Constructor.
  DynamicsRetractor(const EqualityConstraints::shared_ptr &constraints,
                    const RetractParams::shared_ptr &params,
                    std::optional<const KeyVector> basis_keys = {});

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

 protected:
  template <typename CONTAINER>
  static void classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                           KeySet &v_keys, KeySet &ad_keys);

  void updatePriors(const Values &values, const KeySet &keys,
                    NonlinearFactorGraph &graph);
};


/** Factory class used to create retractor. */
class RetractorCreator {
protected:
  RetractParams::shared_ptr params_;

public:
  using shared_ptr = std::shared_ptr<RetractorCreator>;
  RetractorCreator(RetractParams::shared_ptr params) : params_(params) {}

  RetractParams::shared_ptr params() {return params_; }

  virtual ~RetractorCreator() {}

  virtual Retractor::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints) const = 0;
};

class UoptRetractorCreator : public RetractorCreator {
public:
  UoptRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  virtual ~UoptRetractorCreator() {}

  Retractor::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints) const override {
    return std::make_shared<UoptRetractor>(constraints, params_);
  }
};

class ProjRetractorCreator : public RetractorCreator {
public:
  ProjRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  virtual ~ProjRetractorCreator() {}

  Retractor::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints) const override {
    return std::make_shared<ProjRetractor>(constraints, params_);
  }
};

class BasisRetractorCreator : public RetractorCreator {
public:
  BasisKeyFunc basis_key_func_;

public:
  BasisRetractorCreator(
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params) {}

  BasisRetractorCreator(
      BasisKeyFunc basis_key_func,
      RetractParams::shared_ptr params = std::make_shared<RetractParams>())
      : RetractorCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~BasisRetractorCreator() {}

  Retractor::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints) const override {
    KeyVector basis_keys = basis_key_func_(constraints->keyVector());
    return std::make_shared<BasisRetractor>(constraints, params_, basis_keys);
  }
};

class DynamicsRetractorCreator : public RetractorCreator {
protected:
  BasisKeyFunc basis_key_func_;

public:
  DynamicsRetractorCreator(RetractParams::shared_ptr params)
      : RetractorCreator(params) {}

  DynamicsRetractorCreator(RetractParams::shared_ptr params,
                           BasisKeyFunc basis_key_func)
      : RetractorCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~DynamicsRetractorCreator() {}

  Retractor::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints) const override {
    if (params_->use_basis_keys) {
      KeyVector basis_keys = basis_key_func_(constraints->keyVector());
      return std::make_shared<DynamicsRetractor>(constraints, params_, basis_keys);
    }
    else {
      return std::make_shared<DynamicsRetractor>(constraints, params_);
    }
  }
};

}  // namespace gtsam
