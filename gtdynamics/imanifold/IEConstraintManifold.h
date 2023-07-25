/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEConstraintManifold.h
 * @brief Manifold with boundary/corners formulated by only inequality
 * constraints
 * @author: Yetong Zhang
 */

#pragma once

#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/optimizer/InequalityConstraint.h"
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/imanifold/TangentCone.h>
#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

class IEConstraintManifold {
public:
  typedef std::shared_ptr<IEConstraintManifold> shared_ptr;

  struct Params {
    using shared_ptr = std::shared_ptr<Params>;
    ConstraintManifold::Params::shared_ptr ecm_params =
        std::make_shared<ConstraintManifold::Params>();
    // IERetractType ie_retract_type = IERetractType::Barrier;
    IERetractor::shared_ptr retractor;

    /** Default constructor. */
    Params() = default;
  };

protected:
  Params::shared_ptr params_;
  ConnectedComponent::shared_ptr e_cc_;
  gtdynamics::InequalityConstraints::shared_ptr i_constraints_;
  gtsam::Values values_;    // values of variables in CCC
  IndexSet active_indices_; // indices of active i_constraints
  size_t embedding_dim_;
  size_t e_constraints_dim_;
  size_t dim_;
  TspaceBasis::shared_ptr e_basis_;
  TangentCone::shared_ptr i_cone_;

public:
  IEConstraintManifold(
     const Params::shared_ptr &params,
      const ConnectedComponent::shared_ptr &e_cc,
      const gtdynamics::InequalityConstraints::shared_ptr &i_constraints,
      const Values &values, const std::optional<IndexSet> &active_indices = {})
      : params_(params), e_cc_(e_cc), i_constraints_(i_constraints),
        values_(values),
        active_indices_(
            active_indices ? *active_indices
                           : IdentifyActiveConstraints(*i_constraints, values)),
        embedding_dim_(values.dim()),
        e_constraints_dim_(e_cc_->constraints_.dim()),
        dim_(embedding_dim_ - e_constraints_dim_),
        e_basis_(ConstraintManifold::constructTspaceBasis(params->ecm_params,
                                                          e_cc, values, dim_)),
        i_cone_(ConstructTangentCone(*i_constraints, values, active_indices_,
                                     e_basis_)) {}

  IEConstraintManifold createWithNewValues(
      const Values &values,
      const std::optional<IndexSet> &active_indices = {}) const {
    return IEConstraintManifold(params_, e_cc_, i_constraints_, values, active_indices);
  }

  virtual ~IEConstraintManifold() {}

  const Values &values() const { return values_; }

  /// Set of inequality constraints that are currently active.
  const IndexSet &activeIndices() const { return active_indices_; }

  const TspaceBasis::shared_ptr eBasis() const { return e_basis_; }

  const gtdynamics::InequalityConstraints::shared_ptr &iConstraints() const {
    return i_constraints_;
  }

  const ConnectedComponent::shared_ptr &eCC() const { return e_cc_; }

  /// Set of blocking constraints when going in the direction g, return set of
  /// blocking constraint indices, and the projected vector
  virtual std::pair<IndexSet, Vector>
  projectTangentCone(const Vector &xi) const {
    return i_cone_->project(xi);
  }

  /** retract that forces the set of blocking constraints as equality constraints.
   * Will also enforce satisfying other inequality constraints. */
  virtual IEConstraintManifold
  retract(const Vector &xi,
          const std::optional<IndexSet> &blocking_indices = {}) const {
    auto tangent_vector = e_basis_->computeTangentVector(xi);
    return retract(tangent_vector, blocking_indices);
  }

  virtual IEConstraintManifold
  retract(const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const {
    return params_->retractor->retract(this, delta, blocking_indices);
  }

  IEConstraintManifold moveToBoundary(const IndexSet& active_indices) const {
    return params_->retractor->moveToBoundary(this, active_indices);
  }

  /// Construct an e-constraint manifold that only includes equalily
  /// constraints.
  ConstraintManifold eConstraintManifold() const {
    return ConstraintManifold(e_cc_, values_, params_->ecm_params, false,
                              e_basis_);
  }

  /// Construct an equality-constraint-manifold by treating tight constraints as
  /// equality constraints. Used for EQP-based methods.,
  ConstraintManifold eConstraintManifold(const IndexSet &active_indices) const {
    // TODO: construct e-basis using createWithAdditionalConstraints
    gtdynamics::EqualityConstraints active_constraints = e_cc_->constraints_;
    for (const auto &idx : active_indices) {
      active_constraints.emplace_back(
          i_constraints_->at(idx)->createEqualityConstraint());
    }
    auto new_cc =
        std::make_shared<gtsam::ConnectedComponent>(active_constraints);
    return ConstraintManifold(new_cc, values_, params_->ecm_params, false);
    // auto new_basis =
    // e_basis_->createWithAdditionalConstraints(active_constraints); return
    // ConstraintManifold(e_cc_, values_, params_->ecm_params, false,
    // new_basis);
  }

protected:
  static IndexSet IdentifyActiveConstraints(
      const gtdynamics::InequalityConstraints &i_constraints,
      const Values &values) {
    IndexSet active_set;
    for (size_t i = 0; i < i_constraints.size(); i++) {
      if (i_constraints.at(i)->isActive(values)) {
        active_set.insert(i);
      }
    }
    return active_set;
  }

  static TangentCone::shared_ptr
  ConstructTangentCone(const gtdynamics::InequalityConstraints &i_constraints,
                       const Values &values, const IndexSet &active_indices,
                       const TspaceBasis::shared_ptr &t_basis) {
    Matrix A = Matrix::Zero(active_indices.size(), t_basis->dim());
    std::vector<size_t> active_indices_vec(active_indices.begin(),
                                           active_indices.end());

    for (size_t i = 0; i < active_indices_vec.size(); i++) {
      const auto &i_constraint = i_constraints.at(active_indices_vec[i]);
      auto jacobians = i_constraint->jacobians(values);

      for (const Key &key : jacobians.keys()) {
        A.row(i) += jacobians.at(key) * t_basis->recoverJacobian(key);
      }
    }

    auto cone = std::make_shared<TangentCone>(A);
    return cone;
  }
};

} // namespace gtsam
