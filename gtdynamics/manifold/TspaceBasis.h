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

#include <Eigen/Sparse>
#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

// Method to compute tangent space basis
enum BasisType {
  MATRIX = 0,
  SPARSE_MATRIX = 1,
  ELIMINATION = 2,
  SPECIFY_VARIABLES = 3
};

/// Manifold-specific parameters for tangent space basis.
struct TspaceBasisParams {
public:
  using shared_ptr = boost::shared_ptr<TspaceBasisParams>;

  /// Member variables.
  BasisType basis_type = BasisType::MATRIX;
  bool always_construct_basis = true;
  bool use_basis_keys = false;

  /// Constructors.
  TspaceBasisParams() = default;

  /// Functions
  void setMatrix() {
    basis_type = BasisType::MATRIX;
    use_basis_keys = false;
  }

  void setSparseMatrix() {
    basis_type = BasisType::SPARSE_MATRIX;
    use_basis_keys = false;
  }

  void setFixVars() {
    basis_type = BasisType::SPECIFY_VARIABLES;
    use_basis_keys = true;
  }

  void setElimination(bool _use_basis_keys = false) {
    basis_type = BasisType::ELIMINATION;
    use_basis_keys = _use_basis_keys;
  }
};

/// Base class for tangent space basis of constraint manifold.
class TspaceBasis {
protected:
  TspaceBasisParams::shared_ptr params_;
  bool is_constructed_;

public:
  using shared_ptr = boost::shared_ptr<TspaceBasis>;

  /// Default constructor.
  TspaceBasis(
      TspaceBasisParams::shared_ptr params = boost::make_shared<TspaceBasisParams>())
      : params_(params), is_constructed_(false) {}

  static shared_ptr create(const TspaceBasisParams::shared_ptr params,
                           const ConnectedComponent::shared_ptr cc,
                           const Values &values,
                           boost::optional<const KeyVector&> basis_keys = boost::none,
                           boost::optional<size_t> manifold_dim = boost::none);

  /// Default destructor.
  virtual ~TspaceBasis() {}

  /// Construct new basis by using new values
  virtual shared_ptr
  createWithNewValues(const ConnectedComponent::shared_ptr &cc,
                      const Values &values) const = 0;

  /** Compute the tangent vector in the ambient space, given a vector xi
   * representing the magnitude of each basis component. */
  virtual VectorValues computeTangentVector(const Vector &xi) const = 0;

  /** Compute the jacobian of recover function. i.e., function that recovers an
   * original variable from the constraint manifold. */
  virtual Matrix recoverJacobian(const Key &key) const = 0;

  /// Implementation of localCoordinate function for the constraint manifold.
  virtual Vector localCoordinates(const Values &values,
                                  const Values &values_other) const = 0;

  /// Dimension of the basis.
  virtual size_t dim() const = 0;

  /// Construct the actual basis, all the heavy computation goes here.
  virtual void construct(const ConnectedComponent::shared_ptr &cc,
                         const Values &values) = 0;

  /// Return if the basis is already constructed.
  bool isConstructed() const { return is_constructed_; }

  virtual void
  print(const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) = 0;
};

class EmptyBasis : public TspaceBasis {
 public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   */
  EmptyBasis(const TspaceBasisParams::shared_ptr &params) : TspaceBasis(params) {}

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const ConnectedComponent::shared_ptr &cc,
                      const Values &values) const override {
    return boost::make_shared<EmptyBasis>(params_);
  }

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override {
    throwError();
    return VectorValues();
  }

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override {
    throwError();
    return Matrix();
  };

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override {
    throwError();
    return Vector();
  };

  /// Dimension of the basis.
  size_t dim() const override { return 0; }

  /// Construct the actual basis, all the heavy computation goes here.
  void construct(const ConnectedComponent::shared_ptr &cc,
                 const Values &values) override {}

  void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) override {
    std::cout << "Empty basis\n";
  }

  void throwError() const {
    throw std::runtime_error("Empty basis shall not be called");
  }
};

/** Tangent space basis implmented using a matrix, e.g., the kernel of Dh(X),
 * where h(X)=0 represents all the constraints. */
class MatrixBasis : public TspaceBasis {
protected:
  std::map<Key, size_t> var_location_; // location of variables in Jacobian
  std::map<Key, size_t> var_dim_;      // dimension of variables
  size_t total_basis_dim_;
  gtsam::Matrix basis_; // basis for the tangent space

public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   */
  MatrixBasis(const TspaceBasisParams::shared_ptr &params,
              const ConnectedComponent::shared_ptr &cc, const Values &values);

  /// Constructor from other, avoids recomputation.
  MatrixBasis(const ConnectedComponent::shared_ptr &cc, const Values &values,
              const MatrixBasis &other)
      : TspaceBasis(other.params_), var_location_(other.var_location_),
        var_dim_(other.var_dim_), total_basis_dim_(other.total_basis_dim_) {
    if (params_->always_construct_basis) {
      construct(cc, values);
    }
  }

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const ConnectedComponent::shared_ptr &cc,
                      const Values &values) const override {
    return boost::make_shared<MatrixBasis>(cc, values, *this);
  }

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return total_basis_dim_; }

  /// Basis matrix.
  const Matrix &matrix() const { return basis_; }

  /// Construct the actual basis, all the heavy computation goes here.
  void construct(const ConnectedComponent::shared_ptr &cc,
                 const Values &values) override;

  void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) override {
    std::cout << "Matrix basis\n" << matrix() << "\n";
  }
};

/** Tangent space basis implmented using a matrix, e.g., the kernel of Dh(X),
 * where h(X)=0 represents all the constraints. */
class SparseMatrixBasis : public TspaceBasis {
public:
  typedef Eigen::SparseMatrix<double> SpMatrix;
  typedef Eigen::Triplet<double> Triplet;

protected:
  std::map<Key, size_t> var_location_; // location of variables in Jacobian
  std::map<Key, size_t> var_dim_;      // dimension of variables
  size_t total_variable_dim_;
  size_t total_constraint_dim_;
  size_t total_basis_dim_;
  SpMatrix basis_; // basis for the tangent space

public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   */

  SparseMatrixBasis(const TspaceBasisParams::shared_ptr &params,
                    const ConnectedComponent::shared_ptr &cc,
                    const Values &values);

  /// Constructor from other, avoids recomputation.
  SparseMatrixBasis(const ConnectedComponent::shared_ptr &cc,
                    const Values &values, const SparseMatrixBasis &other)
      : TspaceBasis(other.params_), var_location_(other.var_location_),
        var_dim_(other.var_dim_),
        total_variable_dim_(other.total_variable_dim_),
        total_constraint_dim_(other.total_constraint_dim_),
        total_basis_dim_(other.total_basis_dim_) {
    if (params_->always_construct_basis) {
      construct(cc, values);
    }
  }

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const ConnectedComponent::shared_ptr &cc,
                      const Values &values) const override {
    return boost::make_shared<SparseMatrixBasis>(cc, values, *this);
  }

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return total_basis_dim_; }

  /// Basis matrix.
  const SpMatrix &matrix() const { return basis_; }

  /// Construct the actual basis, all the heavy computation goes here.
  void construct(const ConnectedComponent::shared_ptr &cc,
                 const Values &values) override;

  void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) override {
    std::cout << "Matrix basis\n" << matrix() << "\n";
  }

protected:
  void setMatrix(const Matrix &matrix, const size_t &row_offset,
                 const size_t &col_offset,
                 std::vector<Triplet> &triplet_list) const;

  void setSparseEntries(const GaussianFactor::shared_ptr &factor,
                        const size_t row_offset,
                        std::vector<Triplet> &triplet_list) const;
};

/** Tangent space basis as the specified variables, the update on
 * the rest of the variables will be computed through variable elimination.
 * The basis matrix will be in the form of [B;I], which means the
 * corresponding rows to the basis variables form the identity matrix. */
class FixedVarBasis : public TspaceBasis {
protected:
  KeyVector basis_keys_;
  Ordering ordering_;
  size_t total_basis_dim_;
  std::map<Key, size_t> basis_location_;
  std::map<Key, size_t> var_dim_;
  MultiJacobians jacobians_;

public:
  /** Constructor
   * @param cc constraint-connected component for the constraint manifold
   * @param values values of the variables in the connected component
   * @param basis_keys variables selected as basis variables
   */
  FixedVarBasis(const TspaceBasisParams::shared_ptr &params,
                   const ConnectedComponent::shared_ptr &cc,
                   const Values &values,
                   boost::optional<const KeyVector&> basis_keys = boost::none);

  /// Constructor from other, avoids recomputation.
  FixedVarBasis(const ConnectedComponent::shared_ptr &cc,
                   const Values &values, const FixedVarBasis &other);

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const ConnectedComponent::shared_ptr &cc,
                      const Values &values) const override {
    return boost::make_shared<FixedVarBasis>(cc, values, *this);
  }

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /// Implmentation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return total_basis_dim_; }

  /// Construct the actual basis.
  void construct(const ConnectedComponent::shared_ptr &cc,
                 const Values &values) override;

  /// print
  void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) override {
    std::cout << "Elimination basis\n";
    for (const auto &it : jacobians_) {
      std::cout << "\033[1;31mVariable: " << keyFormatter(it.first)
                << "\033[0m\n";
      it.second.print("", keyFormatter);
    }
  }

  /// Return a const reference to jacobians.
  const MultiJacobians &jacobians() const { return jacobians_; }
};

} // namespace gtsam
