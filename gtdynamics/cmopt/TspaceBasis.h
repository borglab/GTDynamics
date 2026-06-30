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
#include <cholmod.h>

#include <SuiteSparseQR.hpp>
#endif
#include <gtdynamics/cmopt/MultiJacobian.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
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

using gtsam::DefaultKeyFormatter;
using gtsam::GaussianFactorGraph;
using gtsam::Key;
using gtsam::KeyFormatter;
using gtsam::KeyVector;
using gtsam::Matrix;
using gtsam::NonlinearEqualityConstraints;
using gtsam::NonlinearFactorGraph;
using gtsam::Ordering;
using gtsam::Values;
using gtsam::Vector;
using gtsam::VectorValues;

typedef std::function<KeyVector(const KeyVector &keys)> BasisKeyFunc;

/**
 * Parameters for tangent-space basis construction.
 *
 * These options select basis style (orthonormal vs. basis-key driven),
 * construction timing, and sparse/dense backend preferences.
 *
 * @see README.md#tangent-basis
 */
struct TspaceBasisParams {
 public:
  using shared_ptr = std::shared_ptr<TspaceBasisParams>;

  /// Member variables.
  bool always_construct_basis = true;
  bool use_basis_keys = false;
  bool use_sparse = false;

  /**
   * Constructor.
   * @param _use_basis_keys If true, basis keys are explicitly selected.
   * @param _always_construct_basis If true, build basis immediately.
   */
  TspaceBasisParams(bool _use_basis_keys = false,
                    bool _always_construct_basis = true)
      : always_construct_basis(_always_construct_basis),
        use_basis_keys(_use_basis_keys) {}
};

/**
 * Abstract interface for constraint-manifold tangent-space bases.
 *
 * Implementations map reduced coordinates to ambient tangent vectors, recover
 * Jacobians for substituted factors, and compute local coordinates.
 *
 * @see README.md#tangent-basis
 */
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

  /**
   * Construct a basis by incorporating additional constraints.
   * @param constraints Additional equality constraints.
   * @param values Current variable values.
   * @param create_from_scratch If true, rebuild basis from scratch.
   * @return New basis object.
   */
  virtual shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints &constraints, const Values &values,
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

  /**
   * Compute local coordinates between two value assignments.
   * @param values Source values.
   * @param values_other Target values.
   * @return Local coordinates in basis parameterization.
   */
  virtual Vector localCoordinates(const Values &values,
                                  const Values &values_other) const = 0;

  /// Dimension of the basis.
  virtual size_t dim() const = 0;

  /// Construct the actual basis, all the heavy computation goes here.
  virtual void construct(const Values &values) = 0;

  /// Return if the basis is already constructed.
  bool isConstructed() const { return is_constructed_; }

  virtual void print(
      const gtsam::KeyFormatter &keyFormatter = DefaultKeyFormatter) = 0;

  virtual std::vector<VectorValues> basisVectors() const;
};

/**
 * Orthonormal null-space basis of the constraint Jacobian.
 *
 * This basis computes `ker(J_H)` either with dense or sparse routines and
 * provides reduced-coordinate parameterization for manifold updates.
 *
 * @see README.md#tangent-basis
 */
class OrthonormalBasis : public TspaceBasis {
 protected:
  // Structural properties that do not change with values.
  struct Attributes {
    NonlinearFactorGraph merit_graph;
    std::map<Key, size_t> var_location;  // location of variables in Jacobian
    std::map<Key, size_t> var_dim;       // dimension of variables
    size_t total_var_dim;
    size_t total_constraint_dim;
    size_t total_basis_dim;
    using shared_ptr = std::shared_ptr<Attributes>;
  };
  Attributes::shared_ptr attributes_;
  gtsam::Matrix basis_;  // basis matrix for the tangent space

  typedef Eigen::SparseMatrix<double> SpMatrix;

 public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param values Current values for variables in the component.
   * @param params Basis construction parameters.
   */
  OrthonormalBasis(const NonlinearEqualityConstraints::shared_ptr &constraints,
                   const Values &values,
                   const TspaceBasisParams::shared_ptr &params);

  /**
   * Constructor from precomputed attributes.
   * @param params Basis construction parameters.
   * @param attributes Precomputed structural attributes.
   */
  OrthonormalBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes)
      : TspaceBasis(params), attributes_(attributes) {}

  /**
   * Constructor from precomputed attributes and matrix.
   * @param params Basis construction parameters.
   * @param attributes Precomputed structural attributes.
   * @param mat Basis matrix.
   */
  OrthonormalBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes, const Matrix &mat)
      : TspaceBasis(params), attributes_(attributes), basis_(mat) {}

  /**
   * Constructor from another basis, reusing structural attributes.
   * @param values Current values.
   * @param other Source basis.
   */
  OrthonormalBasis(const Values &values, const OrthonormalBasis &other)
      : TspaceBasis(other.params_), attributes_(other.attributes_) {
    if (params_->always_construct_basis) {
      construct(values);
    }
  }

  /// Create basis with new values.
  TspaceBasis::shared_ptr createWithNewValues(
      const Values &values) const override {
    return std::make_shared<OrthonormalBasis>(values, *this);
  }

  /**
   * Construct a basis by incorporating additional constraints.
   * @param constraints Additional equality constraints.
   * @param values Current variable values.
   * @param create_from_scratch If true, rebuild basis from scratch.
   * @return New basis object.
   */
  TspaceBasis::shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints &constraints, const Values &values,
      bool create_from_scratch = false) const override;

  /// Construct the actual basis, all the heavy computation goes here.
  void construct(const Values &values) override;

  /// Compute the tangent vector in the ambient space.
  VectorValues computeTangentVector(const Vector &xi) const override;

  /// Compute xi of basis components.
  Vector computeXi(const VectorValues &tangent_vector) const override;

  /// Jacobian of recover function.
  Matrix recoverJacobian(const Key &key) const override;

  /**
   * Compute local coordinates between two value assignments.
   * @param values Source values.
   * @param values_other Target values.
   * @return Local coordinates in basis parameterization.
   */
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

  /**
   * Update basis with additional constraints in dense mode.
   * @param graph Linearized graph of added constraints.
   * @param new_attributes Updated structural attributes.
   * @return New orthonormal basis.
   */
  TspaceBasis::shared_ptr createWithAdditionalConstraintsDense(
      const GaussianFactorGraph &graph,
      const Attributes::shared_ptr &new_attributes) const;

  /**
   * Update basis with additional constraints in sparse mode.
   * @param graph Linearized graph of added constraints.
   * @param new_attributes Updated structural attributes.
   * @return New orthonormal basis.
   */
  TspaceBasis::shared_ptr createWithAdditionalConstraintsSparse(
      const GaussianFactorGraph &graph,
      const Attributes::shared_ptr &new_attributes) const;

  /**
   * Rearrange Jacobian columns to match internal variable ordering.
   * @param A Input Jacobian matrix.
   * @param A_keys Keys corresponding to blocks in `A`.
   * @return Rearranged Jacobian matrix.
   */
  Matrix rearrangeMatrix(const Matrix &A, const KeyVector &A_keys) const;

  Matrix computeConstraintJacobian(const Values &values) const;

  /**
   * Construct transposed sparse Jacobian from sparse triplets.
   * @param nrows Number of rows in original Jacobian.
   * @param ncols Number of cols in original Jacobian (including RHS column).
   * @param triplets Sparse Jacobian triplets.
   * @param cc CHOLMOD context.
   * @return Transposed sparse Jacobian matrix.
   */
#if defined(GTDYNAMICS_WITH_SUITESPARSE)
  static cholmod_sparse *SparseJacobianTranspose(
      const size_t nrows, const size_t ncols,
      const std::vector<std::tuple<int, int, double>> &triplets,
      cholmod_common *cc);

  /**
   * Create sparse selector matrix that picks the last columns.
   * @param nrows Number of rows.
   * @param ncols Number of selected columns.
   * @param cc CHOLMOD context.
   * @return Sparse selector matrix.
   */
  static cholmod_sparse *LastColsSelectionMat(const size_t nrows,
                                              const size_t ncols,
                                              cholmod_common *cc);

  /**
   * Convert CHOLMOD sparse matrix to Eigen sparse matrix.
   * @param A CHOLMOD sparse matrix.
   * @param cc CHOLMOD context.
   * @return Eigen sparse matrix.
   */
  static SpMatrix CholmodToEigen(cholmod_sparse *A, cholmod_common *cc);
#else
  /**
   * Construct transposed sparse Jacobian from sparse triplets.
   * @param nrows Number of rows in original Jacobian.
   * @param ncols Number of cols in original Jacobian (including RHS column).
   * @param triplets Sparse Jacobian triplets.
   * @return Transposed sparse Jacobian matrix.
   */
  static SpMatrix SparseJacobianTranspose(
      const size_t nrows, const size_t ncols,
      const std::vector<std::tuple<int, int, double>> &triplets);

  /**
   * Create sparse selector matrix that picks the last columns.
   * @param nrows Number of rows.
   * @param ncols Number of selected columns.
   * @return Sparse selector matrix.
   */
  static SpMatrix LastColsSelectionMat(const size_t nrows, const size_t ncols);
#endif

  SpMatrix eigenSparseJacobian(const GaussianFactorGraph &graph) const;

  // cholmod_sparse* suiteSparseJacobian(const GaussianFactorGraph &graph);

  /** Return jacobian of a Gaussian factor graph represented as an Eigen sparse
   * matrix. */
  static SpMatrix EigenSparseJacobian(const GaussianFactorGraph &graph);
};

/**
 * Tangent basis parameterized by selected basis variables.
 *
 * `EliminationBasis` represents the tangent space of one
 * constraint-connected component by treating caller-selected variables as the
 * independent coordinates. A reduced tangent vector `xi` is laid out in
 * `basis_keys` order. Basis-variable tangent entries are copied directly from
 * `xi`, while all non-basis variable entries are recovered from Jacobians
 * produced by partially eliminating the constraint graph. Conceptually this is
 * an implicit `[B; I]` basis, where `I` applies to the selected basis keys and
 * `B` maps those coordinates to the eliminated variables.
 *
 * Lifecycle:
 * - The constructor builds structural data once for a component: variable
 *   dimensions, basis-key offsets, the constraint merit graph, and a symbolic
 *   elimination ordering.
 * - If `params->always_construct_basis` is true, the constructor also calls
 *   `construct(values)`. Otherwise construction is lazy and happens when a
 *   caller such as `ConstraintManifold::retract`, `recover` with Jacobians, or
 *   `localCoordinates` first needs the basis.
 * - `createWithNewValues` is used whenever a manifold is rebuilt at new values,
 *   including after retractions and across nonlinear optimization iterations.
 *   It reuses the structural attributes but may reconstruct the numeric
 *   Jacobians depending on `always_construct_basis`.
 *
 * Cost model:
 * - Structural setup is dominated by basis-key bookkeeping and the constrained
 *   COLAMD ordering over the constraint merit graph.
 * - `construct(values)` is the expensive step: it linearizes all equality
 *   constraints in the component, partially eliminates non-basis variables by
 *   QR, and extracts Bayes-net Jacobians. Its cost is graph- and fill-in
 *   dependent, comparable to one sparse linearization/elimination of the
 *   component constraints.
 * - Per-retraction calls to `computeTangentVector` do not relinearize or
 *   eliminate; they only copy basis slices and multiply stored Jacobian blocks,
 *   so their cost is proportional to the stored Jacobian nonzeros.
 *
 * @see README.md#tangent-basis
 * @see README.md#retraction
 */
class EliminationBasis : public TspaceBasis {
 protected:
  struct Attributes {
    NonlinearFactorGraph merit_graph;      ///< Constraint penalty graph.
    KeyVector basis_keys;                  ///< Independent coordinates.
    Ordering ordering;                     ///< Non-basis elimination order.
    size_t total_basis_dim;                ///< Reduced tangent dimension.
    std::map<Key, size_t> basis_location;  ///< Offset of basis key in xi.
    std::map<Key, size_t> var_dim;         ///< Ambient dimension per variable.

    using shared_ptr = std::shared_ptr<Attributes>;
  };
  Attributes::shared_ptr attributes_;  ///< Shared structural data.
  MultiJacobians jacobians_;           ///< Numeric recovery Jacobian blocks.

 public:
  /**
   * Construct an elimination basis for one constraint-connected component.
   *
   * This constructor always builds the structural attributes and, when
   * `params->always_construct_basis` is true, immediately calls
   * `construct(values)`. Explicit `basis_keys` are required by the current
   * implementation; omitting them throws at runtime. The selected keys must
   * have total dimension `values.dim() - constraints->dim()`.
   *
   * Construction-time cost is dominated by the constrained COLAMD ordering; if
   * immediate numeric construction is enabled, add the cost of one constraint
   * linearization and QR partial elimination.
   *
   * @param constraints Equality constraints for the component.
   * @param values Values of variables in the connected component.
   * @param params Basis construction parameters.
   * @param basis_keys Variables selected as independent basis coordinates.
   */
  EliminationBasis(const NonlinearEqualityConstraints::shared_ptr &constraints,
                   const Values &values,
                   const TspaceBasisParams::shared_ptr &params,
                   std::optional<const KeyVector> basis_keys = {});

  /**
   * Recreate a basis at new values while reusing structural attributes.
   *
   * This is the path used by `createWithNewValues`, which is in turn called
   * when `ConstraintManifold` or `IEConstraintManifold` creates a new manifold
   * state after retraction or between nonlinear iterations. It avoids
   * recomputing basis-key offsets and the symbolic ordering. If
   * `always_construct_basis` is true, it still recomputes the numeric recovery
   * Jacobians at `values`.
   *
   * @param values Current values for the same component structure.
   * @param other Source basis providing shared structural attributes.
   */
  EliminationBasis(const Values &values, const EliminationBasis &other);

  /**
   * Construct from precomputed structural attributes.
   *
   * This internal constructor is used by `createWithAdditionalConstraints`
   * after it has assembled updated structural attributes. It does not construct
   * numeric Jacobians; the caller either invokes `construct(values)` or assigns
   * a composed Jacobian cache.
   *
   * @param params Basis construction parameters.
   * @param attributes Shared structural data for the new basis.
   */
  EliminationBasis(const TspaceBasisParams::shared_ptr &params,
                   const Attributes::shared_ptr &attributes)
      : TspaceBasis(params), attributes_(attributes) {}

  /**
   * Create a basis at new values with the same structural attributes.
   *
   * This is called when a manifold is rebuilt after a retraction or nonlinear
   * trial/update. It is cheap when `always_construct_basis` is false, and
   * otherwise costs one call to `construct(values)`.
   *
   * @param values Current values for the same component structure.
   * @return Basis sharing structure with this basis and updated numerics if
   * requested by the parameters.
   */
  TspaceBasis::shared_ptr createWithNewValues(
      const Values &values) const override {
    return std::make_shared<EliminationBasis>(values, *this);
  }

  /**
   * Construct a basis by incorporating additional constraints.
   *
   * This is used by inequality-constrained manifolds when active inequalities
   * are temporarily converted into equality constraints. Basis keys that appear
   * in the additional constraints are removed from the independent coordinate
   * set because those directions are no longer free.
   *
   * If `create_from_scratch` is true, the new basis pays the full
   * `construct(values)` cost. Otherwise it composes the existing recovery
   * Jacobians with a lightweight Jacobian map for the new active constraints,
   * which avoids a fresh graph linearization/elimination but still costs a
   * multiplication over the stored Jacobian blocks.
   *
   * @param constraints Additional equality constraints.
   * @param values Current variable values.
   * @param create_from_scratch If true, rebuild basis from scratch.
   * @return New basis object.
   */
  TspaceBasis::shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints &constraints, const Values &values,
      bool create_from_scratch = false) const override;

  /**
   * Lift reduced coordinates into an ambient tangent vector.
   *
   * This is called during retractions and optimizer linear/nonlinear update
   * paths. It is a per-call operation, often executed many times per nonlinear
   * iteration/trial. It assumes the numeric basis has already been constructed;
   * callers that need lazy construction should call through
   * `ConstraintManifold::makeSureBasisConstructed`.
   *
   * Cost is proportional to the selected basis dimension plus the stored
   * recovery Jacobian nonzeros. It does not linearize factors or eliminate.
   *
   * @param xi Reduced tangent coordinates in basis-key order.
   * @return Ambient tangent vector for all variables represented by this basis.
   */
  VectorValues computeTangentVector(const Vector &xi) const override;

  /**
   * Extract reduced coordinates from an ambient tangent vector.
   *
   * Only the basis-key entries are read; non-basis entries are ignored because
   * they are dependent coordinates. Cost is linear in the total basis
   * dimension.
   *
   * @param tangent_vector Ambient tangent vector.
   * @return Reduced coordinates in basis-key order.
   */
  Vector computeXi(const VectorValues &tangent_vector) const override;

  /**
   * Return the recover Jacobian for a variable.
   *
   * `ConstraintManifold::recover(key, H)` and transformed-factor
   * linearization call this when derivatives with respect to manifold
   * coordinates are needed. It allocates a dense
   * `var_dim(key) x total_basis_dim` matrix and fills the blocks stored for
   * `key`; it does not relinearize or eliminate.
   *
   * @param key Variable key to recover.
   * @return Jacobian of the recovered variable local coordinates w.r.t. `xi`.
   */
  Matrix recoverJacobian(const Key &key) const override;

  /**
   * Compute local coordinates between two value assignments.
   *
   * This is called by manifold `localCoordinates`, typically while evaluating
   * optimizer deltas between two manifold states. It computes local coordinates
   * only for basis keys because those are the independent coordinates. Cost is
   * the sum of the basis-key local-coordinate costs.
   *
   * @param values Source values.
   * @param values_other Target values.
   * @return Local coordinates in basis parameterization.
   */
  Vector localCoordinates(const Values &values,
                          const Values &values_other) const override;

  /// Dimension of the basis.
  size_t dim() const override { return attributes_->total_basis_dim; }

  /**
   * Construct numeric recovery Jacobians at the supplied values.
   *
   * This linearizes the constraint merit graph, partially eliminates the
   * non-basis variables with QR, and extracts Bayes-net Jacobians. It is the
   * expensive numeric step and may run at basis construction time, lazily on
   * first use, or when a new manifold state is created, depending on
   * `always_construct_basis`.
   *
   * @param values Current linearization point.
   */
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

/**
 * Factory interface for creating tangent-space basis instances.
 *
 * @see README.md#tangent-basis
 */
class TspaceBasisCreator {
 protected:
  TspaceBasisParams::shared_ptr params_;

 public:
  using shared_ptr = std::shared_ptr<TspaceBasisCreator>;
  TspaceBasisCreator(TspaceBasisParams::shared_ptr params) : params_(params) {}

  virtual ~TspaceBasisCreator() {}

  /**
   * Create a basis object for a component.
   * @param constraints Equality constraints for the component.
   * @param values Current values for variables in the component.
   * @return Created basis object.
   */
  virtual TspaceBasis::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const Values &values) const = 0;
};

/**
 * Factory for `OrthonormalBasis`.
 *
 * @see README.md#tangent-basis
 */
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

  TspaceBasis::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const Values &values) const override {
    return std::make_shared<OrthonormalBasis>(constraints, values, params_);
  }
};

/**
 * Factory for `EliminationBasis`.
 *
 * This is the normal construction path used by `ConstraintManifold` params. At
 * manifold construction time, `create` receives the keys in one
 * constraint-connected component, applies `basis_key_func_`, and creates an
 * `EliminationBasis` with those independent coordinates. The factory itself is
 * cheap; all graph-ordering and numeric construction costs are paid by the
 * basis object it creates.
 *
 * @see README.md#tangent-basis
 */
class EliminationBasisCreator : public TspaceBasisCreator {
 protected:
  BasisKeyFunc basis_key_func_;

 public:
  /**
   * Construct a factory without an explicit key selector.
   *
   * This constructor is retained for API compatibility, but normal callers
   * should use the constructor that accepts a `BasisKeyFunc`. With the default
   * parameters, `create` needs a key selector; with `use_basis_keys=false`, the
   * current `EliminationBasis` implementation reaches the no-key path, which
   * is not implemented.
   *
   * @param params Basis construction parameters.
   */
  EliminationBasisCreator(TspaceBasisParams::shared_ptr params =
                              std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params) {}

  /**
   * Create an elimination-basis factory with custom key selector.
   * @param basis_key_func Callback selecting basis keys.
   * @param params Basis construction parameters.
   */
  EliminationBasisCreator(BasisKeyFunc basis_key_func,
                          TspaceBasisParams::shared_ptr params =
                              std::make_shared<TspaceBasisParams>(true))
      : TspaceBasisCreator(params), basis_key_func_(basis_key_func) {}

  virtual ~EliminationBasisCreator() {}

  /**
   * Create basis for one constraint-connected component.
   *
   * This is executed when a `ConstraintManifold` or `IEConstraintManifold`
   * creates the tangent basis for a component. It is not called on every
   * tangent lift, but it may be called again when a new manifold object is
   * created for updated values or active constraints. The returned basis
   * controls whether numeric construction happens immediately or lazily through
   * `TspaceBasisParams::always_construct_basis`.
   *
   * @param constraints Equality constraints for the component.
   * @param values Current values for variables in the component.
   * @return Created elimination basis.
   */
  TspaceBasis::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const Values &values) const override {
    if (params_->use_basis_keys) {
      KeyVector basis_keys = basis_key_func_(values.keys());
      return std::make_shared<EliminationBasis>(constraints, values, params_,
                                                basis_keys);
    }
    return std::make_shared<EliminationBasis>(constraints, values, params_);
  }
};

}  // namespace gtdynamics
