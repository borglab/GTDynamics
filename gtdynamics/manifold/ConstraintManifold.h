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

#include <cstddef>
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/Retractor.h>
#include <gtdynamics/manifold/TspaceBasis.h>

namespace gtsam {

/// Function to find the basis keys for constraint manifold.
// typedef KeyVector (*BasisKeyFunc)(const ConnectedComponent::shared_ptr &);

typedef std::function<KeyVector (const ConnectedComponent::shared_ptr & cc)> BasisKeyFunc;

/** Manifold representing constraint-connected component. Any element on the
 * manifold is the values of variables in CCC satisfying the constraints, e.g.,
 * {X : h(X)=0}. */
class ConstraintManifold {
public:
  /** Parameters for constraint manifold. */
  struct Params {
    using shared_ptr = boost::shared_ptr<Params>;

    // Member variables.
    RetractParams::shared_ptr retract_params =
        boost::make_shared<RetractParams>();
    TspaceBasisParams::shared_ptr basis_params =
        boost::make_shared<TspaceBasisParams>();
    BasisKeyFunc basis_key_func = NULL;

    /** Default constructor. */
    Params() = default;
  };

protected:
  Params::shared_ptr params_;
  ConnectedComponent::shared_ptr cc_;
  Retractor::shared_ptr retractor_; // retraction operation
  gtsam::Values values_;            // values of variables in CCC
  size_t embedding_dim_;            // dimension of embedding space
  size_t constraint_dim_;           // dimension of constriants
  size_t dim_;                      // dimension of constraint manifold
  TspaceBasis::shared_ptr basis_;   // tangent space basis

public:
  enum { dimension = Eigen::Dynamic };

  typedef OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> ChartJacobian;

  /** Constructor from Connected Component.
   * @param cc      constraint-connected component
   * @param values  values of variable in the connected component
   * @param params  parameters for constraint manifold
   * @param retract_init  whether to perform retract in initialization
   */
  ConstraintManifold(
      const ConnectedComponent::shared_ptr &cc, const gtsam::Values &values,
      const Params::shared_ptr &params = boost::make_shared<Params>(),
      bool retract_init = true)
      : params_(params), cc_(cc), retractor_(constructRetractor(params, cc)),
        values_(constructValues(cc, values, retractor_, retract_init)),
        embedding_dim_(values_.dim()), constraint_dim_(cc->constraints_.dim()),
        dim_(embedding_dim_ > constraint_dim_ ? embedding_dim_ - constraint_dim_
                                              : 0),
        basis_(constructTspaceBasis(params, cc, values_, dim_)) {}

  /** constructor from other manifold but update the values. */
  ConstraintManifold(const ConstraintManifold &other, const Values &values)
      : params_(other.params_), cc_(other.cc_), retractor_(other.retractor_),
        values_(values), embedding_dim_(other.embedding_dim_),
        constraint_dim_(other.constraint_dim_), dim_(other.dim_),
        basis_(other.basis_->createWithNewValues(cc_, values_)) {}

  /** Construct new ConstraintManifold with new values. Note: this function
   * indirectly calls retractConstraints. */
  ConstraintManifold createWithNewValues(const gtsam::Values &values) const {
    return ConstraintManifold(*this, values);
  }

  /// Dimension of the constraint manifold.
  inline size_t dim() const { return dim_; }

  /// Base values of the CCC.
  inline const Values &values() const { return values_; }

  /// Get base value with optional Jacobian.
  const gtsam::Value &recover(const gtsam::Key key,
                              ChartJacobian H1 = boost::none) const;

  /// Get base value by type with optional Jacobian.
  template <typename ValueType>
  inline const ValueType &recover(const gtsam::Key key,
                                  ChartJacobian H1 = boost::none) const {
    return recover(key, H1).cast<ValueType>();
  }

  /** Retraction of the constraint manifold, e.g., retraction required for gtsam
   * manifold type. Note: Jacobians are set as zero since they are not required
   * for optimization. */
  ConstraintManifold retract(const gtsam::Vector &xi,
                             ChartJacobian H1 = boost::none,
                             ChartJacobian H2 = boost::none) const;

  /** LocalCoordinates of the constraint manifold, e.g., localCoordinates
   * required for gtsam manifold type. */
  gtsam::Vector localCoordinates(const ConstraintManifold &g,
                                 ChartJacobian H1 = boost::none,
                                 ChartJacobian H2 = boost::none) const;

  /// print
  void print(const std::string &s = "") const;

  /// equals
  bool equals(const ConstraintManifold &other, double tol = 1e-8) const;

  /// Return the basis of the tangent space.
  const TspaceBasis::shared_ptr &basis() const { return basis_; }

  /// Return the retractor.
  const Retractor::shared_ptr &retractor() const { return retractor_; }

  const Values feasibleValues() const;

protected:
  /** Initialize the values_ of variables in CCC and compute dimension of the
   * constraint manifold and compute the dimension of the constraint manifold.
   */
  static Values constructValues(const ConnectedComponent::shared_ptr cc,
                                const gtsam::Values &values,
                                const Retractor::shared_ptr &retractor,
                                bool retract_init);

  /// Make sure the tangent space basis is constructed.
  void makeSureBasisConstructed() const {
    if (!basis_->isConstructed()) {
      basis_->construct(cc_, values_);
    }
  }

  /// Construct the retractor used to perform retraction.
  static Retractor::shared_ptr
  constructRetractor(const Params::shared_ptr &params,
                     const ConnectedComponent::shared_ptr &cc);

  /// Construct the basis for tangent space.
  static TspaceBasis::shared_ptr
  constructTspaceBasis(const Params::shared_ptr &params,
                       const ConnectedComponent::shared_ptr &cc,
                       const Values &values, size_t manifold_dim);
};

// Specialize ConstraintManifold traits to use a Retract/Local
template <>
struct traits<ConstraintManifold>
    : gtsam::internal::Manifold<ConstraintManifold> {};

} // namespace gtsam
