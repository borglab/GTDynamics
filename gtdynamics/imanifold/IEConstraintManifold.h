/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEConstraintManifold.h
 * @brief Manifold with boundary/corners formulated by equality and inequality
 * constraints
 * @author: Yetong Zhang
 */

#pragma once

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
    IERetractorCreator::shared_ptr retractor_creator;

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
  IERetractor::shared_ptr retractor_;

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
                                     e_basis_)) {
    retractor_ = params->retractor_creator->create(*this);
  }

  /** constructor from other manifold but update the values. */
  IEConstraintManifold(const IEConstraintManifold &other, const Values &values,
                       const std::optional<IndexSet> &active_indices = {})
      : params_(other.params_), e_cc_(other.e_cc_),
        i_constraints_(other.i_constraints_), values_(values),
        active_indices_(active_indices ? *active_indices
                                       : IdentifyActiveConstraints(
                                             *i_constraints_, values)),
        embedding_dim_(other.embedding_dim_),
        e_constraints_dim_(other.e_constraints_dim_), dim_(other.dim_),
        e_basis_(other.e_basis_->createWithNewValues(e_cc_, values_)),
        i_cone_(ConstructTangentCone(*i_constraints_, values_, active_indices_,
                                     e_basis_)),
        retractor_(other.retractor_) {}

  IEConstraintManifold createWithNewValues(
      const Values &values,
      const std::optional<IndexSet> &active_indices = {}) const {
    return IEConstraintManifold(*this, values, active_indices);
  }

  virtual ~IEConstraintManifold() {}

  const Values &values() const { return values_; }

  /// Set of inequality constraints that are currently active.
  const IndexSet &activeIndices() const { return active_indices_; }

  const TspaceBasis::shared_ptr eBasis() const { return e_basis_; }

  const TangentCone::shared_ptr &tangentCone() const {return i_cone_; }

  const IERetractor::shared_ptr &retractor() const {return retractor_; }

  const gtdynamics::InequalityConstraints::shared_ptr &iConstraints() const {
    return i_constraints_;
  }

  const ConnectedComponent::shared_ptr &eCC() const { return e_cc_; }

  const size_t &dim() const  {return dim_; }

  /// Set of blocking constraints when going in the direction g, return set of
  /// blocking constraint indices, and the projected vector
  virtual std::pair<IndexSet, Vector>
  projectTangentCone(const Vector &xi) const;

  virtual std::pair<IndexSet, VectorValues>
  projectTangentCone(const VectorValues &tangent_vector) const;

  /** retract that forces the set of blocking constraints as equality
   * constraints. Will also enforce satisfying other inequality constraints. */
  virtual IEConstraintManifold
  retract(const Vector &xi,
          const std::optional<IndexSet> &blocking_indices = {}) const;

  virtual IEConstraintManifold
  retract(const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {}) const;

  IndexSet blockingIndices(const VectorValues &tangent_vector) const;

  IEConstraintManifold moveToBoundary(const IndexSet &active_indices) const;

  /// Construct an e-constraint manifold that only includes equalily
  /// constraints.
  ConstraintManifold eConstraintManifold() const;

  /// Construct an equality-constraint-manifold by treating tight constraints as
  /// equality constraints. Used for EQP-based methods.,
  ConstraintManifold eConstraintManifold(const IndexSet &active_indices) const;

  double evalIViolation() const {
    return i_constraints_->evaluateViolationL2Norm(values_);
  }

  double evalEViolation() const {
    return e_cc_->constraints_.evaluateViolationL2Norm(values_); 
  }

protected:
  /// Identify the current active inequality constraints.
  static IndexSet IdentifyActiveConstraints(
      const gtdynamics::InequalityConstraints &i_constraints,
      const Values &values);

  /** Construct the tangent cone with the following steps:
   * 1) linearize active i-constraints
   * 2) compute jacobians on t-space basis
   * 3) use jacobian matrix to construct tangent cone
   */
  static TangentCone::shared_ptr
  ConstructTangentCone(const gtdynamics::InequalityConstraints &i_constraints,
                       const Values &values, const IndexSet &active_indices,
                       const TspaceBasis::shared_ptr &t_basis);
};

} // namespace gtsam