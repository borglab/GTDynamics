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

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmcopt/TangentCone.h>
#include <gtdynamics/cmopt/ConstraintManifold.h>

namespace gtdynamics {
using namespace gtsam;


class IEConstraintManifold {
public:
  typedef std::shared_ptr<IEConstraintManifold> shared_ptr;

  struct Params {
    using shared_ptr = std::shared_ptr<Params>;
    ConstraintManifold::Params::shared_ptr equalityManifoldParams =
        std::make_shared<ConstraintManifold::Params>();
    // IERetractType ie_retract_type = IERetractType::Barrier;
    IERetractorCreator::shared_ptr retractorCreator;
    TangentSpaceBasisCreator::shared_ptr equalityBasisCreator;
    bool equalityBasisBuildFromScratch = true;
    /** Default constructor. */
    Params() = default;
  };

protected:
  Params::shared_ptr params_;
  NonlinearEqualityConstraints::shared_ptr e_constraints_;
  NonlinearInequalityConstraints::shared_ptr i_constraints_;
  Values values_;    // values of variables in CCC
  IndexSet active_indices_; // indices of active i_constraints
  size_t embedding_dim_;
  size_t e_constraints_dim_;
  size_t dim_;
  TangentSpaceBasis::shared_ptr e_basis_;
  TangentCone::shared_ptr i_cone_;
  IERetractor::shared_ptr retractor_;

public:
  /// Bundle one constraint-connected component as the feasible set from thesis
  /// Eq. (4.34), M_IE = {x | h(x)=0, g(x)>=0}, plus its active set.
  IEConstraintManifold(
      const Params::shared_ptr &params,
      const NonlinearEqualityConstraints::shared_ptr &e_constraints,
      const NonlinearInequalityConstraints::shared_ptr &i_constraints,
      const Values &values, const std::optional<IndexSet> &active_indices = {})
      : params_(params), e_constraints_(e_constraints),
        i_constraints_(i_constraints), values_(values),
        active_indices_(
            identifyActiveConstraints(*i_constraints, values, active_indices)),
        embedding_dim_(values.dim()), e_constraints_dim_(e_constraints_->dim()),
        dim_(embedding_dim_ - e_constraints_dim_),
        e_basis_(params->equalityBasisCreator->create(e_constraints_, values_)),
        i_cone_(constructTangentCone(*i_constraints, values, active_indices_,
                                     e_basis_)),
        retractor_(params->retractorCreator->create(*this)) {}

  /** constructor from other manifold but update the values. */
  IEConstraintManifold(const IEConstraintManifold &other, const Values &values,
                       const std::optional<IndexSet> &active_indices = {})
      : params_(other.params_), e_constraints_(other.e_constraints_),
        i_constraints_(other.i_constraints_), values_(values),
        active_indices_(
            identifyActiveConstraints(*i_constraints_, values, active_indices)),
        embedding_dim_(other.embedding_dim_),
        e_constraints_dim_(other.e_constraints_dim_), dim_(other.dim_),
        e_basis_(other.e_basis_->createWithNewValues(values_)),
        i_cone_(constructTangentCone(*i_constraints_, values_, active_indices_,
                                     e_basis_)),
        retractor_(other.retractor_) {}

  IEConstraintManifold createWithNewValues(
      const Values &values,
      const std::optional<IndexSet> &active_indices = {}) const {
    return IEConstraintManifold(*this, values, active_indices);
  }

  virtual ~IEConstraintManifold() {}

  const Values &values() const { return values_; }

  /// Active set A(x): tight inequality indices, as in thesis Eq. (4.3).
  const IndexSet &activeIndices() const { return active_indices_; }

  const TangentSpaceBasis::shared_ptr eBasis() const { return e_basis_; }

  /// Tangent cone C_x induced by thesis Eqs. (4.14)-(4.16).
  const TangentCone::shared_ptr &tangentCone() const { return i_cone_; }

  const IERetractor::shared_ptr &retractor() const { return retractor_; }

  const NonlinearInequalityConstraints::shared_ptr &iConstraints() const {
    return i_constraints_;
  }

  const NonlinearEqualityConstraints::shared_ptr &eConstraints() const {
    return e_constraints_;
  }

  const size_t &dim() const { return dim_; }

  /// Project xi onto the cone as in thesis Eq. (4.31)/(4.42), using the
  /// active linearized inequalities in manifold coordinates.
  virtual std::pair<IndexSet, Vector>
  projectTangentCone(const Vector &xi) const;

  /// Same projection as above, but starting from ambient tangent vectors.
  virtual std::pair<IndexSet, VectorValues>
  projectTangentCone(const VectorValues &tangentVector) const;

  /// Retract using the on-corner rule from thesis Eq. (4.46) when blocking
  /// inequalities are supplied.
  virtual IEConstraintManifold
  retract(const Vector &xi,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const;

  /// Retract an ambient tangent update back to a feasible IE manifold point.
  virtual IEConstraintManifold
  retract(const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const;

  /// Return active inequalities that block the step, matching thesis Eq. (4.45).
  IndexSet blockingIndices(const VectorValues &tangentVector) const;

  /// Force a mode change by moving directly onto the requested boundary set.
  IEConstraintManifold
  moveToBoundary(const IndexSet &active_indices,
                 IERetractionInfo *retract_info = nullptr) const;

  /// Construct an e-constraint manifold that only includes equalily
  /// constraints.
  ConstraintManifold eConstraintManifold() const;

  /// Construct an equality-constraint-manifold by treating tight constraints as
  /// equality constraints. Used for EQP-based methods.,
  ConstraintManifold eConstraintManifold(const IndexSet &active_indices) const;

  double evaluateInequalityViolation() const {
    return i_constraints_->violationNorm(values_);
  }

  double evaluateEqualityViolation() const {
    return e_constraints_->violationNorm(values_);
  }

  /// Linearize active inequalities as Dg_A(x) B_x, the K matrix in thesis
  /// Eq. (4.15), for manifold-space QPs.
  LinearIConstraintMap
  linearActiveManifoldInequalityConstraints(const Key manifold_key) const;

  /// Linearize the active inequalities directly in ambient variable space.
  LinearIConstraintMap linearActiveBaseInequalityConstraints() const;

protected:
  /// Identify A(x), either from explicit mode information or from which
  /// inequalities are tight at the current values.
  static IndexSet identifyActiveConstraints(
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values, const std::optional<IndexSet> &active_indices = {});

  /// Construct the tangent cone in thesis Eqs. (4.14)-(4.16) by forming the
  /// active Jacobian Dg_A(x) and projecting it through the equality basis B_x.
  static TangentCone::shared_ptr
  constructTangentCone(const NonlinearInequalityConstraints &i_constraints,
                       const Values &values, const IndexSet &active_indices,
                       const TangentSpaceBasis::shared_ptr &t_basis);
};

class IEManifoldValues : public std::map<Key, IEConstraintManifold> {
public:
  using base = std::map<Key, IEConstraintManifold>;
  using base::base;

  Values baseValues() const;

  KeyVector keys() const;

  IEManifoldValues
  moveToBoundaries(const IndexSetMap &approach_indices_map) const;

  // VectorValues computeTangentVector(const VectorValues &delta) const;

  // EManifoldValues retract(const VectorValues &delta) const;
};

} // namespace gtdynamics
