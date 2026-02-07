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
#if defined(GTDYNAMICS_WITH_SUITESPARSE)
#include <SuiteSparseQR.hpp>
#include <cholmod.h>
#endif
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>

namespace gtdynamics {

using EqualityConstraints = gtsam::NonlinearEqualityConstraints;
using gtsam::DefaultKeyFormatter;
using gtsam::GaussianFactorGraph;
using gtsam::Key;
using gtsam::KeyFormatter;
using gtsam::KeyVector;
using gtsam::Matrix;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Values;
using gtsam::Vector;
using gtsam::VectorValues;

typedef std::function<KeyVector(const KeyVector &keys)> BasisKeyFunc;

/// Manifold-specific parameters for tangent space basis.
struct TspaceBasisParams {
public:
  using shared_ptr = std::shared_ptr<TspaceBasisParams>;

  /// Member variables.
  bool always_construct_basis = true;
  bool use_basis_keys = false;
  bool use_sparse = false;

  /// Constructors.
  TspaceBasisParams(bool _use_basis_keys = false,
                    bool _always_construct_basis = true)
      : always_construct_basis(_always_construct_basis),
        use_basis_keys(_use_basis_keys) {}
};

/// Base class for tangent space basis of constraint manifold.
class TspaceBasis {
protected:
  TspaceBasisParams::shared_ptr params_;
  bool is_constructed_;

public:
  using shared_ptr = std::shared_ptr<TspaceBasis>;

  /// Default constructor.
  TspaceBasis(TspaceBasisParams::shared_ptr params =
                  std::make_shared<TspaceBasisParams>())
      : params_(params), is_constructed_(false) {}

  /// Default destructor.
  virtual ~TspaceBasis() {}

  /// Construct new basis by using new values
  virtual shared_ptr createWithNewValues(const Values &values) const = 0;

  /// Construct new basis by incorporating new constraints
  virtual shared_ptr
  createWithAdditionalConstraints(const EqualityConstraints &constraints,
                                  const Values &values,
                                  bool create_from_scratch = false) const = 0;

  /** Compute the tangent vector in the ambient space, given a vector xi
   * representing the magnitude of each basis component. */
  virtual VectorValues computeTangentVector(const Vector &xi) const = 0;

  /** Given a tangent vector, compute a vector xi representing the magnitude of
   * basis components. */
  virtual Vector computeXi(const VectorValues &tangent_vector) const = 0;

  /** Compute the jacobian of recover function. i.e., function that recovers an
   * original variable from the constraint manifold. */
  virtual Matrix recoverJacobian(const Key &key) const = 0;

  /// Implementation of localCoordinate function for the constraint manifold.
  virtual Vector localCoordinates(const Values &values,
                                  const Values &values_other) const = 0;

  /// Dimension of the basis.
  virtual size_t dim() const = 0;

  /// Construct the actual basis, all the heavy computation goes here.
  virtual void construct(const Values &values) = 0;

  /// Return if the basis is already constructed.
  bool isConstructed() const { return is_constructed_; }

  virtual void
  print(const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) = 0;

  virtual std::vector<VectorValues> basisVectors() const;
};

/** Orthonormal space basis computed as the kernel of the constraint jacobian
 * matrix, e.g., the kernel of Dh(X), where h(X)=0 represents all the
 * constraints. Two options are provided as either using dense matrix or sparse
 * matrix for computing the kernel space. */
class OrthonormalBasis : public TspaceBasis {
protected:
  // Structural properties that do not change with values.
  struct Attributes {
    NonlinearFactorGraph merit_graph;
    std::map<Key, size_t> var_location; // location of variables in Jacobian
    std::map<Key, size_t> var_dim;      // dimension of variables
    size_t total_var_dim;
    size_t total_constraint_dim;
    size_t total_basis_dim;
    using shared_ptr = std::shared_ptr<Attributes>;
  };
  Attributes::shared_ptr attributes_;
  gtsam::Matrix basis_; // basis matrix for the tangent space

  typedef Eigen::SparseMatrix<double> SpMatrix;

public:
  OrthonormalBasis(const EqualityConstraints::shared_ptr &constraints,
                   const Values &values,
                   const TspaceBasisParams::shared_ptr &params);

  OrthonormalBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes)
      : TspaceBasis(params), attributes_(attributes) {}

  OrthonormalBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes, const Matrix &mat)
      : TspaceBasis(params), attributes_(attributes), basis_(mat) {}

  /// Constructor from other, avoids recomputation.
  OrthonormalBasis(const Values &values, const OrthonormalBasis &other)
      : TspaceBasis(other.params_), attributes_(other.attributes_) {
    if (params_->always_construct_basis) {
      construct(values);
    }
  }

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const Values &values) const override {
    return std::make_shared<OrthonormalBasis>(values, *this);
  }

  /// Construct new basis by incorporating new constraints
  TspaceBasis::shared_ptr createWithAdditionalConstraints(
      const EqualityConstraints &constraints, const Values &values,
      bool create_from_scratch = false) const override;

  /// Construct the actual basis, all the heavy computation goes here.
  void construct(const Values &values) override;

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Compute xi of basis components.
  Vector computeXi(const VectorValues &tangent_vector) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /// Implementation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return attributes_->total_basis_dim; }

  /// Basis matrix.
  const Matrix &matrix() const { return basis_; }

  void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) override {
    std::cout << "Orthonormal basis\n" << matrix() << "\n";
  }

protected:
  void constructDense(const Values &values);

  void constructSparse(const Values &values);

  TspaceBasis::shared_ptr createWithAdditionalConstraintsDense(
      const GaussianFactorGraph &graph,
      const Attributes::shared_ptr &new_attributes) const;

  TspaceBasis::shared_ptr createWithAdditionalConstraintsSparse(
      const GaussianFactorGraph &graph,
      const Attributes::shared_ptr &new_attributes) const;

  Matrix rearrangeMatrix(const Matrix &A, const KeyVector &A_keys) const;

  Matrix computeConstraintJacobian(const Values &values) const;

  /** Construct the transposed jacobian from sparse jacobian triplets. */
#if defined(GTDYNAMICS_WITH_SUITESPARSE)
  static cholmod_sparse *SparseJacobianTranspose(
      const size_t nrows, const size_t ncols,
      const std::vector<std::tuple<int, int, double>> &triplets,
      cholmod_common *cc);

  static cholmod_sparse *LastColsSelectionMat(const size_t nrows,
                                              const size_t ncols,
                                              cholmod_common *cc);

  static SpMatrix CholmodToEigen(cholmod_sparse *A, cholmod_common *cc);
#else
  static SpMatrix SparseJacobianTranspose(
      const size_t nrows, const size_t ncols,
      const std::vector<std::tuple<int, int, double>> &triplets);

  static SpMatrix LastColsSelectionMat(const size_t nrows, const size_t ncols);
#endif

  SpMatrix eigenSparseJacobian(const GaussianFactorGraph &graph) const;

  // cholmod_sparse* suiteSparseJacobian(const GaussianFactorGraph &graph);

  /** Return jacobian of a Gaussian factor graph represented as an Eigen sparse
   * matrix. */
  static SpMatrix EigenSparseJacobian(const GaussianFactorGraph &graph);
};

/** Tangent space basis as the specified variables, the update on
 * the rest of the variables will be computed through variable elimination.
 * The basis matrix will be in the form of [B;I], which means the
 * corresponding rows to the basis variables form the identity matrix. */
class EliminationBasis : public TspaceBasis {
protected:
  struct Attributes {
    NonlinearFactorGraph merit_graph;
    KeyVector basis_keys;
    Ordering ordering;
    size_t total_basis_dim;
    std::map<Key, size_t> basis_location; // location of basis var in xi
    std::map<Key, size_t> var_dim;

    using shared_ptr = std::shared_ptr<Attributes>;
  };
  Attributes::shared_ptr attributes_;
  MultiJacobians jacobians_;

public:
  /** Constructor
   * @param constraints constraints for the constraint manifold
   * @param values values of the variables in the connected component
   * @param basis_keys variables selected as basis variables
   */
  EliminationBasis(const EqualityConstraints::shared_ptr &constraints,
                   const Values &values,
                   const TspaceBasisParams::shared_ptr &params,
                   std::optional<const KeyVector> basis_keys = {});

  /// Constructor from other, avoids recomputation.
  EliminationBasis(const Values &values, const EliminationBasis &other);

  EliminationBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes)
      : TspaceBasis(params), attributes_(attributes) {}

  /// Create basis with new values.
  TspaceBasis::shared_ptr
  createWithNewValues(const Values &values) const override {
    return std::make_shared<EliminationBasis>(values, *this);
  }

  /// Construct new basis by incorporating new constraints
  TspaceBasis::shared_ptr createWithAdditionalConstraints(
      const EqualityConstraints &constraints, const Values &values,
      bool create_from_scratch = false) const override;

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Compute xi of basis components.
  Vector computeXi(const VectorValues &tangent_vector) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /// Implementation of localCoordinate for the constraint manifold.
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return attributes_->total_basis_dim; }

  /// Construct the actual basis.
  void construct(const Values &values) override;

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

/** Factory class used to create Tspace basis. */
class TspaceBasisCreator {
protected:
  TspaceBasisParams::shared_ptr params_;

public:
  using shared_ptr = std::shared_ptr<TspaceBasisCreator>;
  TspaceBasisCreator(TspaceBasisParams::shared_ptr params) : params_(params) {}

  virtual ~TspaceBasisCreator() {}

  virtual TspaceBasis::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints,
         const Values &values) const = 0;
};

class OrthonormalBasisCreator : public TspaceBasisCreator {
public:
  OrthonormalBasisCreator(TspaceBasisParams::shared_ptr params =
                              std::make_shared<TspaceBasisParams>())
      : TspaceBasisCreator(params) {}

  static TspaceBasisCreator::shared_ptr CreateSparse() {
    auto params = std::make_shared<TspaceBasisParams>();
    params->use_sparse = true;
    return std::make_shared<OrthonormalBasisCreator>(params);
  }

  virtual ~OrthonormalBasisCreator() {}

  TspaceBasis::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints,
         const Values &values) const override {
    return std::make_shared<OrthonormalBasis>(constraints, values, params_);
  }
};

class EliminationBasisCreator : public TspaceBasisCreator {
protected:
  BasisKeyFunc basis_key_func_;

public:
  EliminationBasisCreator(TspaceBasisParams::shared_ptr params =
                              std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params) {}

  EliminationBasisCreator(BasisKeyFunc basis_key_func,
                          TspaceBasisParams::shared_ptr params =
                              std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~EliminationBasisCreator() {}

  TspaceBasis::shared_ptr
  create(const EqualityConstraints::shared_ptr constraints,
         const Values &values) const override {
    if (params_->use_basis_keys) {
      KeyVector basis_keys = basis_key_func_(values.keys());
      return std::make_shared<EliminationBasis>(constraints, values, params_,
                                                basis_keys);
    }
    return std::make_shared<EliminationBasis>(constraints, values, params_);
  }
};

} // namespace gtdynamics
