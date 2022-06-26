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

#include <gtdynamics/optimizer/ConnectedComponent.h>
#include <gtdynamics/optimizer/MultiJacobian.h>

namespace gtsam {

/// Manifold-specific parameters for tangent space basis.
struct BasisParams {
 public:
  using shared_ptr = boost::shared_ptr<BasisParams>;
  KeyVector basis_keys_;
  BasisParams() {}
  BasisParams(const KeyVector& basis_keys) : basis_keys_(basis_keys) {}
};

/// Base class for tangent space basis of constraint manifold.
class TspaceBasis {
 public:
  using shared_ptr = boost::shared_ptr<TspaceBasis>;

  /// Default constructor.
  TspaceBasis() {}

  /** Compute the tangent vector in the ambient space, given a vector xi
   * representing the magnitude of each basis component. */
  virtual VectorValues computeTangentVector(const Vector& xi) const = 0;

  /** Compute the jacobian of recover function. i.e., function that recovers an
   * original variable from the constraint manifold. */
  virtual Matrix recoverJacobian(const Key& key) const = 0;

  /// Implementation of localCoordinate function for the constraint manifold.
  virtual Vector localCoordinates(const Values& values,
                                  const Values& values_other) const = 0;

  /// Dimension of the basis.
  virtual const size_t& dim() const = 0;

  /// Retract on the base variables.
  virtual Values retractBaseVariables(const Values& values, const Vector& xi) {
    return values.retract(computeTangentVector(xi));
  }
};

/** Tangent space basis implmented using a matrix, e.g., the kernel of Dh(X),
 * where h(X)=0 represents all the constraints. */
class MatrixBasis : public TspaceBasis {
 protected:
  gtsam::Matrix basis_;                 // basis for the tangent space
  std::map<Key, size_t> var_location_;  // location of variables in Jacobian
  std::map<Key, size_t> var_dim_;       // dimension of variables
  size_t total_basis_dim_;

 public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   */
  MatrixBasis(const ConnectedComponent::shared_ptr& cc, const Values& values);

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector& xi) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key& key) const override;

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values& values,
                          const Values& values_other) const override;

  /// Dimension of the basis.
  const size_t& dim() const override { return total_basis_dim_; }

  /// Basis matrix.
  const Matrix& matrix() const { return basis_; }
};

/** Tangent space basis as the specified variables, the update on
 * the rest of the variables will be computed through variable elimination.
 * The basis matrix will be in the form of [B;I], which means the
 * corresponding rows to the basis variables form the identity matrix. */
class EliminationBasis : public TspaceBasis {
 protected:
  KeyVector basis_keys_;
  MultiJacobians jacobians_;
  size_t total_basis_dim_;
  std::map<Key, size_t> basis_location_;
  std::map<Key, size_t> var_dim_;

 public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   * @param basis_keys variables selected as basis variables
   */
  EliminationBasis(const ConnectedComponent::shared_ptr& cc,
                   const Values& values, const KeyVector& basis_keys);

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector& xi) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key& key) const override;

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values& values,
                          const Values& values_other) const override;

  /// Dimension of the basis.
  const size_t& dim() const override { return total_basis_dim_; }
};

}  // namespace gtsam
