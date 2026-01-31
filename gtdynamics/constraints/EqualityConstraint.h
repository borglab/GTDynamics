/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  EqualityConstraint.h
 * @brief Equality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

/**
 * Equality constraint base class.
 */
class EqualityConstraint {
 public:
  typedef EqualityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  EqualityConstraint() {}

  /** Destructor. */
  virtual ~EqualityConstraint() {}

  /** @brief Return tolerance as a vector. */
  virtual gtsam::Vector tolerance() const = 0;

  /**
   * @brief Create a factor representing the component in the merit function.
   *
   * @param mu penalty parameter.
   * @param bias additional bias.
   * @return a factor representing 1/2 mu||g(x)+bias||_Diag(tolerance^2)^2.
   */
  virtual gtsam::NoiseModelFactor::shared_ptr createFactor(
      const double mu, std::optional<gtsam::Vector> bias = {}) const = 0;

  /** @brief Create a factor with constrained noise model. */
  virtual gtsam::NoiseModelFactor::shared_ptr createConstrainedFactor() const;

  /**
   * @brief Check if constraint violation is within tolerance.
   *
   * @param x values to evalute constraint at.
   * @return bool representing if is feasible.
   */
  virtual bool feasible(const gtsam::Values& x) const = 0;

  /**
   * @brief Evaluate the constraint violation, g(x).
   *
   * @param x values to evalute constraint at.
   * @return a vector representing the constraint violation in each dimension.
   */
  virtual gtsam::Vector operator()(const gtsam::Values& x) const = 0;

  gtsam::Vector evaluate(const gtsam::Values& x) const;

  /** @brief Constraint violation scaled by tolerance, e.g. g(x)/tolerance. */
  virtual gtsam::Vector toleranceScaledViolation(
      const gtsam::Values& x) const = 0;

  /** @brief return the dimension of the constraint. */
  virtual size_t dim() const = 0;

  /// Return keys of variables involved in the constraint.
  virtual std::set<gtsam::Key> keys() const { return std::set<gtsam::Key>(); }
};

/** Equality constraint that force g(x) = 0, where g(x) is a scalar-valued
 * function. */
class DoubleExpressionEquality : public EqualityConstraint {
 public:
 protected:
  gtsam::Expression<double> expression_;
  double tolerance_;

 public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param tolerance   scalar representing tolerance.
   */
  DoubleExpressionEquality(const gtsam::Expression<double>& expression,
                           const double& tolerance)
      : expression_(expression), tolerance_(tolerance) {}

  /** @brief Return tolerance as a vector. */
  gtsam::Vector tolerance() const override {
    return gtsam::Vector1(tolerance_);
  }

  /** Create a factor representing the component in the merit function. */
  gtsam::NoiseModelFactor::shared_ptr createFactor(
      const double mu, std::optional<gtsam::Vector> bias = {}) const override;

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values& x) const override;

  /** Evaluate the constraint violation, g(x). */
  gtsam::Vector operator()(const gtsam::Values& x) const override;

  /** Constraint violation scaled by tolerance, e.g. g(x)/tolerance. */
  gtsam::Vector toleranceScaledViolation(const gtsam::Values& x) const override;

  /** Return the dimension of the constraint. */
  size_t dim() const override { return 1; }

  std::set<gtsam::Key> keys() const override { return expression_.keys(); }
};

/** Equality constraint that force g(x) = 0, where g(x) is a vector-valued
 * function. */
template <int P>
class VectorExpressionEquality : public EqualityConstraint {
 public:
  using VectorP = Eigen::Matrix<double, P, 1>;

 protected:
  gtsam::Expression<VectorP> expression_;
  VectorP tolerance_;

 public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param tolerance   vector representing tolerance in each dimension.
   */
  VectorExpressionEquality(const gtsam::Expression<VectorP>& expression,
                           const VectorP& tolerance)
      : expression_(expression), tolerance_(tolerance) {}

  /** @brief Return tolerance as a vector. */
  gtsam::Vector tolerance() const override {
    return tolerance_;
  }

  /** Create a factor representing the component in the merit function. */
  gtsam::NoiseModelFactor::shared_ptr createFactor(
      const double mu, std::optional<gtsam::Vector> bias = {}) const override;

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values& x) const override;

  /** Evaluate the constraint violation, g(x). */
  gtsam::Vector operator()(const gtsam::Values& x) const override;

  /** Constraint violation scaled by tolerance, e.g. g(x)/tolerance. */
  gtsam::Vector toleranceScaledViolation(const gtsam::Values& x) const override;

  /** Return the dimension of the constraint. */
  size_t dim() const override;

  std::set<gtsam::Key> keys() const override { return expression_.keys(); }
};

/** Equality constraint that force factor error to be 0. */
class FactorZeroErrorConstraint : public EqualityConstraint {
 protected:
  gtsam::NoiseModelFactor::shared_ptr factor_;
  gtsam::Vector tolerance_;

 public:
  /**
   * @brief Constructor.
   *
   * @param factor  NoiseModel factor.
   * @param tolerance   vector representing tolerance in each dimension.
   */
  FactorZeroErrorConstraint(const gtsam::NoiseModelFactor::shared_ptr& factor,
                            const gtsam::Vector& tolerance)
      : factor_(factor), tolerance_(tolerance) {}

  /** @brief Return tolerance as a vector. */
  gtsam::Vector tolerance() const override {
    return tolerance_;
  }

  /// Constructor from a NoiseModelFactor, and use the noise model as tolerance.
  FactorZeroErrorConstraint(const gtsam::NoiseModelFactor::shared_ptr& factor)
      : FactorZeroErrorConstraint(factor, factor->noiseModel()->sigmas()) {}

  /** Create a factor representing the component in the merit function. */
  gtsam::NoiseModelFactor::shared_ptr createFactor(
      const double mu, std::optional<gtsam::Vector> bias = {}) const override;

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values& x) const override;

  /** Evaluate the constraint violation, g(x). */
  gtsam::Vector operator()(const gtsam::Values& x) const override;

  /** Constraint violation scaled by tolerance, e.g. g(x)/tolerance. */
  gtsam::Vector toleranceScaledViolation(const gtsam::Values& x) const override;

  /** Return the dimension of the constraint. */
  size_t dim() const override { return factor_->dim(); }

  /// Return keys involved in constraint.
  std::set<gtsam::Key> keys() const override {
    std::set<gtsam::Key> keyset;
    for (const gtsam::Key& key: factor_->keys()) {
        keyset.insert(key);
    }
    return keyset;
    }
};

/// Container of EqualityConstraint.
class EqualityConstraints : public std::vector<EqualityConstraint::shared_ptr> {
 public:
    using shared_ptr = std::shared_ptr<EqualityConstraints>;
 private:
  using Base = std::vector<EqualityConstraint::shared_ptr>;

  template <typename DERIVEDCONSTRAINT>
  using IsDerived = typename std::enable_if<
      std::is_base_of<EqualityConstraint, DERIVEDCONSTRAINT>::value>::type;

 public:
  EqualityConstraints() : Base() {}

  /// Add a set of equality constraints.
  void add(const EqualityConstraints& other) {
    insert(end(), other.begin(), other.end());
  }

  /// Emplace a shared pointer to constraint of given type.
  template <class DERIVEDCONSTRAINT, class... Args>
  IsDerived<DERIVEDCONSTRAINT> emplace_shared(Args&&... args) {
    push_back(std::allocate_shared<DERIVEDCONSTRAINT>(
        Eigen::aligned_allocator<DERIVEDCONSTRAINT>(),
        std::forward<Args>(args)...));
  }

  /// Return the total dimension of constraints.
  size_t dim() const;

  /// Return keys involved in constraints.
  gtsam::KeySet keys() const;

  gtsam::KeyVector keyVector() const;

  /// Evaluate the constraint violation (as L2 norm).
  double evaluateViolationL2Norm(const gtsam::Values &values) const;

  gtsam::VariableIndex varIndex() const;

  gtsam::NonlinearFactorGraph meritGraph(const double mu = 1.0) const;

  gtsam::NonlinearFactorGraph constrainedGraph() const;
};

/// Create FactorZeroErrorConstraintConstraints from the factors of a graph.
EqualityConstraints ConstraintsFromGraph(
    const gtsam::NonlinearFactorGraph& graph);

typedef std::function<KeyVector(const KeyVector &keys)> BasisKeyFunc;

}  // namespace gtsam

#include <gtdynamics/constraints/EqualityConstraint-inl.h>
