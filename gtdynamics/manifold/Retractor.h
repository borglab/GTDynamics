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
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

// Method to perform retraction
enum RetractType { UOPT = 0, PROJ = 1, FIX_VARS = 2, DYNAMICS = 3 };

/// Parameters for constraint manifold retraction operation.
struct RetractParams {
 public:
  using shared_ptr = std::shared_ptr<RetractParams>;

  // Member variables
  bool check_feasible = false;
  double feasible_threshold = 1e-5;
  LevenbergMarquardtParams lm_params;
  RetractType retract_type = RetractType::UOPT;
  bool use_basis_keys = false;
  double sigma = 1.0;
  bool apply_base_retraction = false;
  bool recompute = false;

  // Constructor
  RetractParams() = default;

  // Functions.
  void setUopt() {
    retract_type = RetractType::UOPT;
    use_basis_keys = false;
  }

  void setProjection(bool _use_basis_keys = false, double _sigma = 1.0,
                     bool _apply_base_retraction = false) {
    retract_type = RetractType::PROJ;
    use_basis_keys = _use_basis_keys;
    sigma = _sigma;
    apply_base_retraction = _apply_base_retraction;
  }

  void setFixVars() {
    retract_type = RetractType::FIX_VARS;
    use_basis_keys = true;
  }

  void setDynamics(bool _use_basis_keys = true, double _sigma = 1e-2) {
    retract_type = RetractType::DYNAMICS;
    use_basis_keys = _use_basis_keys;
    sigma = _sigma;
  }
};

/** Base class that implements the retraction operation for the constraint
 * manifold. */
class Retractor {
 protected:
  ConnectedComponent::shared_ptr cc_;
  RetractParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<Retractor>;

  /// Default constructor.
  Retractor(const ConnectedComponent::shared_ptr &cc,
            const RetractParams::shared_ptr &params =
                std::make_shared<RetractParams>())
      : cc_(cc), params_(params) {}

  /// Convenient constructor.
  static shared_ptr create(const RetractParams::shared_ptr &params,
                           const ConnectedComponent::shared_ptr &cc,
                           std::optional<const KeyVector> basis_keys = {});

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

  /** Retraction operation
   * @param values base values composing the constraint manifold
   * @param delta tangent vector
   * @param metric_sigmas sigmas that define the metric for projection
   * @return retracted point on the manifold
   */
  virtual Values retract(const Values &values, const VectorValues &delta,
                         const VectorValues &metric_sigmas) {
    return retract(values, delta);
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
  UoptRetractor(const ConnectedComponent::shared_ptr &cc,
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
  KeyVector basis_keys_;

 public:
  /// Constructor.
  ProjRetractor(const ConnectedComponent::shared_ptr &cc,
                const RetractParams::shared_ptr &params,
                std::optional<const KeyVector> basis_keys = {});

  /// Retraction operation.
  Values retract(const Values &values, const VectorValues &delta) override;

  Values retract(const Values &values, const VectorValues &delta,
                         const VectorValues &metric_sigmas) override;

  Values retractConstraints(const Values &values) override;
};

/** Retractor by specifying the basis variables. */
class BasisRetractor : public Retractor {
 protected:
  KeyVector basis_keys_;
  MutableLMOptimizer optimizer_;
  ConnectedComponent::shared_ptr cc_;
  std::vector<std::shared_ptr<ConstVarFactor>> factors_with_fixed_vars_;

 public:
  /// Constructor.
  BasisRetractor(const ConnectedComponent::shared_ptr &cc,
                 const RetractParams::shared_ptr &params,
                 const KeyVector &basis_keys);

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

 protected:
  void constructGraph(const ConnectedComponent::shared_ptr &cc,
                      const KeyVector &basis_keys);
};

/** Customized retractor for kinodynamics manifold. */
class DynamicsRetractor : public Retractor {
 protected:
  MutableLMOptimizer optimizer_wp_q_, optimizer_wp_v_, optimizer_wp_ad_;
  MutableLMOptimizer optimizer_np_q_, optimizer_np_v_, optimizer_np_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

 public:
  /// Constructor.
  DynamicsRetractor(const ConnectedComponent::shared_ptr &cc,
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

}  // namespace gtsam
