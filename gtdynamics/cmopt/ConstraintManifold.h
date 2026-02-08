/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstraintManifold.h
 * @brief Manifold representing variables satisfying constraints.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/cmopt/TspaceBasis.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

namespace gtdynamics {

using gtsam::Key;
using gtsam::KeyVector;
using gtsam::OptionalJacobian;
using gtsam::Value;
using gtsam::Values;
using gtsam::Vector;
using gtsam::VectorValues;

using gtsam::NonlinearEqualityConstraints;

/**
 * Manifold for one constraint-connected component (CCC).
 *
 * This type stores the original variables that belong to one CCC and exposes
 * manifold operations (`retract`, `localCoordinates`, and `recover`) so the
 * CCC can be optimized as a single manifold-valued variable in transformed
 * optimization problems.
 *
 * The manifold dimension is `embedding_dim - constraint_dim`, and tangent
 * space mapping is delegated to `TspaceBasis` while feasibility projection is
 * delegated to `Retractor`.
 *
 * @see README.md#constraint-manifold
 * @see README.md#tangent-basis
 * @see README.md#retraction
 */
class ConstraintManifold {
 public:
  /**
   * Parameters that define basis construction and retraction behavior.
   *
   * `basis_creator` controls tangent-space parameterization and
   * `retractor_creator` controls how feasibility is enforced after updates.
   *
   * @see README.md#tangent-basis
   * @see README.md#retraction
   */
  struct Params {
    using shared_ptr = std::shared_ptr<Params>;
    TspaceBasisCreator::shared_ptr basis_creator;
    RetractorCreator::shared_ptr retractor_creator;

    /** Default constructor. */
    Params()
        : basis_creator(std::make_shared<OrthonormalBasisCreator>()),
          retractor_creator(std::make_shared<UoptRetractorCreator>()) {}
  };

 protected:
  Params::shared_ptr params_;
  NonlinearEqualityConstraints::shared_ptr constraints_;
  Retractor::shared_ptr retractor_;  // retraction operation
  gtsam::Values values_;             // values of variables in CCC
  size_t embedding_dim_;             // dimension of embedding space
  size_t constraint_dim_;            // dimension of constraints
  size_t dim_;                       // dimension of constraint manifold
  TspaceBasis::shared_ptr basis_;    // tangent space basis

 public:
  enum { dimension = Eigen::Dynamic };

  typedef OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> ChartJacobian;

  /**
   * Constructor from a constraint-connected component.
   * @param constraints Equality constraints defining the component.
   * @param values Initial values of variables in the connected component.
   * @param params Parameters controlling basis and retraction behavior.
   * @param retract_init If true, retract values to satisfy constraints at
   * construction.
   * @param basis Optional pre-built tangent basis.
   */
  ConstraintManifold(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const gtsam::Values &values,
      const Params::shared_ptr &params = std::make_shared<Params>(),
      bool retract_init = true,
      std::optional<TspaceBasis::shared_ptr> basis = {})
      : params_(params),
        constraints_(constraints),
        retractor_(params->retractor_creator->create(constraints_)),
        values_(constructValues(values, retractor_, retract_init)),
        embedding_dim_(values_.dim()),
        constraint_dim_(constraints_->dim()),
        dim_(embedding_dim_ > constraint_dim_ ? embedding_dim_ - constraint_dim_
                                              : 0),
        basis_(basis ? *basis
                     : (dim_ == 0 ? createFixedBasis(constraints_, values_)
                                  : params->basis_creator->create(constraints_,
                                                                  values_))) {}

  /**
   * Construct from another manifold instance with updated values.
   * @param other Source manifold providing structural data.
   * @param values New values for variables in the component.
   */
  ConstraintManifold(const ConstraintManifold &other, const Values &values)
      : params_(other.params_),
        constraints_(other.constraints_),
        retractor_(other.retractor_),
        values_(values),
        embedding_dim_(other.embedding_dim_),
        constraint_dim_(other.constraint_dim_),
        dim_(other.dim_),
        basis_(other.basis_->createWithNewValues(values_)) {}

  /**
   * Construct a new manifold with updated values.
   * @param values New values for variables in the component.
   * @return A new manifold instance with updated basis/retraction state.
   * @note This indirectly calls `retractConstraints`.
   */
  ConstraintManifold createWithNewValues(const gtsam::Values &values) const {
    return ConstraintManifold(*this, values);
  }

  /// Dimension of the constraint manifold.
  inline size_t dim() const { return dim_; }

  /// Base values of the CCC.
  inline const Values &values() const { return values_; }

  /**
   * Recover the value of a base variable.
   * @param key Variable key in the connected component.
   * @param H1 Optional Jacobian of recover operation w.r.t. manifold
   * coordinates.
   * @return Reference to the recovered base value.
   */
  const gtsam::Value &recover(const gtsam::Key key,
                              ChartJacobian H1 = {}) const;

  /**
   * Recover the typed value of a base variable.
   * @tparam ValueType Value type to cast to.
   * @param key Variable key in the connected component.
   * @param H1 Optional Jacobian of recover operation w.r.t. manifold
   * coordinates.
   * @return Reference to the recovered typed value.
   */
  template <typename ValueType>
  inline const ValueType &recover(const gtsam::Key key,
                                  ChartJacobian H1 = {}) const {
    return recover(key, H1).cast<ValueType>();
  }

  /**
   * Retract this manifold element with a tangent update.
   * @param xi Tangent-space coordinates.
   * @param H1 Optional Jacobian w.r.t. current state (not implemented).
   * @param H2 Optional Jacobian w.r.t. tangent update (not implemented).
   * @return Retracted manifold element.
   * @note Jacobians are not implemented and throw if requested.
   */
  ConstraintManifold retract(const gtsam::Vector &xi, ChartJacobian H1 = {},
                             ChartJacobian H2 = {}) const;

  /**
   * Compute local coordinates from this manifold element to another.
   * @param g Target manifold element.
   * @param H1 Optional Jacobian w.r.t. this manifold element (not implemented).
   * @param H2 Optional Jacobian w.r.t. target manifold element
   * (not implemented).
   * @return Tangent-space coordinates from `*this` to `g`.
   */
  gtsam::Vector localCoordinates(const ConstraintManifold &g,
                                 ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  /// print
  void print(const std::string &s = "") const;

  /**
   * Check manifold equality by comparing base values.
   * @param other Manifold to compare with.
   * @param tol Absolute comparison tolerance.
   * @return True if values are equal within tolerance.
   */
  bool equals(const ConstraintManifold &other, double tol = 1e-8) const;

  /// Return the basis of the tangent space.
  const TspaceBasis::shared_ptr &basis() const { return basis_; }

  /// Return the retractor.
  const Retractor::shared_ptr &retractor() const { return retractor_; }

  /// Return values projected onto the feasible set by LM on penalty graph.
  const Values feasibleValues() const;

 protected:
  static TspaceBasis::shared_ptr createFixedBasis(
      const NonlinearEqualityConstraints::shared_ptr &constraints,
      const gtsam::Values &values) {
    auto basis_params = std::make_shared<TspaceBasisParams>();
    basis_params->always_construct_basis = false;
    return std::make_shared<OrthonormalBasis>(constraints, values,
                                              basis_params);
  }

  /**
   * Initialize values for the manifold state.
   * @param values Candidate values of variables in the connected component.
   * @param retractor Retraction object used for feasibility projection.
   * @param retract_init If true, perform constraint retraction.
   * @return Initialized values, optionally retracted to feasibility.
   */
  static Values constructValues(const gtsam::Values &values,
                                const Retractor::shared_ptr &retractor,
                                bool retract_init);

  /// Make sure the tangent space basis is constructed.
  /// NOTE: The static mutex creates a global synchronization point across all
  /// ConstraintManifold instances. This is a known limitation that could cause
  /// unnecessary contention. A better design would move the mutex to the
  /// TspaceBasis class itself.
  void makeSureBasisConstructed() const {
    if (!basis_->isConstructed()) {
      static std::mutex basis_mutex;
      std::lock_guard<std::mutex> lock(basis_mutex);
      if (!basis_->isConstructed()) {
        basis_->construct(values_);
      }
    }
  }
};

/**
 * Container of manifold values keyed by manifold variable keys.
 *
 * This utility wraps a map of `ConstraintManifold` objects and provides helper
 * operations used during optimization, including flattening to base values and
 * lifting deltas to base tangent vectors.
 *
 * @see README.md#constraint-manifold
 * @see README.md#retraction
 */
class EManifoldValues : public std::map<Key, ConstraintManifold> {
 public:
  using base = std::map<Key, ConstraintManifold>;
  using base::base;

  /// Collect all base variables from manifold values.
  Values baseValues() const;

  /// Return keys of manifold variables.
  KeyVector keys() const;

  /// Lift per-manifold delta vectors to base tangent vectors.
  VectorValues computeTangentVector(const VectorValues &delta) const;

  /// Retract each manifold value by the corresponding update in `delta`.
  EManifoldValues retract(const VectorValues &delta) const;

  /// Return manifold dimensions keyed by manifold variable.
  std::map<Key, size_t> dims() const;
};

}  // namespace gtdynamics

namespace gtsam {
template <>
struct traits<gtdynamics::ConstraintManifold>
    : gtsam::internal::Manifold<gtdynamics::ConstraintManifold> {};
}  // namespace gtsam
