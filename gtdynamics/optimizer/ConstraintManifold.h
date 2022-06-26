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

#include <gtdynamics/optimizer/ConnectedComponent.h>
#include <gtdynamics/optimizer/Retractor.h>
#include <gtdynamics/optimizer/TspaceBasis.h>

namespace gtsam {

/** Manifold representing constraint-connected component. Any element on the
 * manifold is the values of variables in CCC satisfying the constraints, e.g.,
 * {X : h(X)=0}. */
class ConstraintManifold {
 public:
  /** Parameters for constraint manifold. */
  struct Params {
    using shared_ptr = boost::shared_ptr<Params>;

    // Method to compute tangent space basis
    enum BasisType { KERNEL = 0, ELIMINATION = 1, SPECIFY_VARIABLES = 2 };
    // Method to perform retraction
    enum RetractType { UOPT = 0, PROJ = 1, PARTIAL_PROJ = 2 };

    // Member variables.
    LevenbergMarquardtParams lm_params;
    RetractType retract_type;
    BasisType basis_type;

    /** Default constructor. */
    Params()
        : lm_params(),
          retract_type(RetractType::UOPT),
          basis_type(BasisType::KERNEL) {}
  };

 protected:
  Params::shared_ptr params_;
  BasisParams::shared_ptr basis_params_;
  ConnectedComponent::shared_ptr cc_;
  Retractor::shared_ptr retractor_;  // retraction operation
  size_t embedding_dim_;             // dimension of embedding space
  size_t constraint_dim_;            // dimension of constriants
  size_t dim_;                       // dimension of constraint manifold
  gtsam::Values values_;             // values of variables in CCC
  TspaceBasis::shared_ptr basis_;    // tangent space basis

 public:
  enum { dimension = Eigen::Dynamic };

  typedef OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> ChartJacobian;

  /** Constructor from Connected Component.
   * @param cc      constraint-connected component
   * @param values  values of variable in the connected component
   * @param retract_init  whether to perform retract in initialization
   * @param construct_basis compute basis on initialization
   * @param basis_keys (optional) manually specified basis variables
   */
  ConstraintManifold(
      const ConnectedComponent::shared_ptr& cc, const gtsam::Values& values,
      const Params::shared_ptr& params = boost::make_shared<Params>(),
      bool retract_init = true, bool construct_basis = true,
      const BasisParams::shared_ptr& basis_params =
          boost::make_shared<BasisParams>())
      : params_(params),
        basis_params_(basis_params),
        cc_(cc),
        retractor_(constructRetractor(params, basis_params, cc)) {
    initializeValues(values);
    if (dim() > 0) {
      if (retract_init) {
        values_ = retractConstraints(values_);
      }
      if (construct_basis) {
        computeBasis();
      }
    }
  }

  /** constructor from other manifold but update the values. */
  ConstraintManifold(const ConstraintManifold& other, const Values& values,
                     const bool retract_init)
      : params_(other.params_),
        basis_params_(other.basis_params_),
        cc_(other.cc_),
        retractor_(other.retractor_),
        embedding_dim_(other.embedding_dim_),
        constraint_dim_(other.constraint_dim_),
        dim_(other.dim_),
        values_(values) {
    if (retract_init) {
      values_ = retractConstraints(values_);
    }
    computeBasis();
  }

  /** Construct new ConstraintManifold with new values. Note: this function
   * indirectly calls retractConstraints. */
  ConstraintManifold createWithNewValues(const gtsam::Values& values,
                                         bool retract_init = true) const {
    return ConstraintManifold(*this, values, retract_init);
  }

  /// Dimension of the constraint manifold.
  inline size_t dim() const { return dim_; }

  /// Base values of the CCC.
  inline const Values& values() const { return values_; }

  /// Get base value with optional Jacobian.
  const gtsam::Value& recover(const gtsam::Key key,
                              ChartJacobian H1 = boost::none) const;

  /// Get base value by type with optional Jacobian.
  template <typename ValueType>
  inline ValueType recover(const gtsam::Key key,
                           ChartJacobian H1 = boost::none) const {
    return recover(key, H1).cast<ValueType>();
  }

  /** Retraction of the constraint manifold, e.g., retraction required for gtsam
   * manifold type. Note: Jacobians are set as zero since they are not required
   * for optimization. */
  ConstraintManifold retract(const gtsam::Vector& xi,
                             ChartJacobian H1 = boost::none,
                             ChartJacobian H2 = boost::none) const;

  /** LocalCoordinates of the constraint manifold, e.g., localCoordinates
   * required for gtsam manifold type. */
  gtsam::Vector localCoordinates(const ConstraintManifold& g,
                                 ChartJacobian H1 = boost::none,
                                 ChartJacobian H2 = boost::none) const;

  /** Given values of variables in CCC that may violate the constraints, compute
   * the values that satisfy the constraints. */
  Values retractConstraints(const Values& values) const;

  /// print
  void print(const std::string& s = "") const;

  /// equals
  bool equals(const ConstraintManifold& other, double tol = 1e-8) const;

  /// Return the basis of the tangent space.
  const TspaceBasis::shared_ptr& basis() const { return basis_; }

  /// Compute the tangent space basis for the constraint manifold.
  void computeBasis();

 protected:
  /** Initialize the values_ of variables in CCC and compute dimension of the
   * constraint manifold and compute the dimension of the constraint manifold.
   */
  void initializeValues(const gtsam::Values& values);

  static Retractor::shared_ptr constructRetractor(
      const Params::shared_ptr& params,
      const BasisParams::shared_ptr& basis_params,
      const ConnectedComponent::shared_ptr& cc);

  // /** Perform retraction by minimizing the constraint violation, e.g.,
  //  * ||h(x)||^2. */
  // gtsam::Values retractUopt(const gtsam::Values& values) const;

  // /** Perform retraction by performing metric projection, e.g., minimizing
  //  * ||dist(x,x0)||^2  s.t. h(x)=0. */
  // gtsam::Values retractProj(const gtsam::Values& values) const;

  // /** Perform retraction by minimizing the constraint violation while fixing
  // the
  //  * specified variables, e.g., min ||h(x)||^2.  s.t. x_s=x0_s. */
  // gtsam::Values retractPProj(const gtsam::Values& values) const;
};

// Specialize ConstraintManifold traits to use a Retract/Local
template <>
struct traits<ConstraintManifold>
    : gtsam::internal::Manifold<ConstraintManifold> {};

}  // namespace gtsam
