/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  InequalityConstraint.h
 * @brief Equality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#pragma once

#include <gtdynamics/factors/BarrierFactor.h>
#include <gtdynamics/factors/SmoothBarrierFactor.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtdynamics {
/**
 * Linear inequality constraint base class.
 */
class LinearInequalityConstraint {
public:
  typedef LinearInequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  LinearInequalityConstraint() {}

  virtual ~LinearInequalityConstraint() {}

  virtual size_t dim() const = 0;

  virtual gtsam::Vector operator()(const gtsam::VectorValues &x) const = 0;

  virtual bool feasible(const gtsam::VectorValues &x,
                        double threshold = 0) const;

  virtual bool isActive(const gtsam::VectorValues &x,
                        double threshold = 1e-5) const {
    return (*this)(x).norm() < threshold;
  }

  virtual gtsam::JacobianFactor::shared_ptr createL2Factor() const = 0;

  virtual gtsam::JacobianFactor::shared_ptr createConstrainedFactor() const = 0;

  virtual gtsam::MultiJacobian jacobian() const = 0;

  virtual void print(const gtsam::KeyFormatter &key_formatter =
                         gtdynamics::GTDKeyFormatter) const;
};

/**
 * Linear inequality constraint represented with a Jacobian factor.
 */
class JacobianLinearInequalityConstraint : public LinearInequalityConstraint {
protected:
  using base = LinearInequalityConstraint;
  gtsam::JacobianFactor::shared_ptr factor_;

public:
  JacobianLinearInequalityConstraint(
      const gtsam::JacobianFactor::shared_ptr &factor)
      : base(), factor_(factor) {}

  size_t dim() const override { return factor_->rows(); }

  gtsam::Vector operator()(const gtsam::VectorValues &x) const override {
    return factor_->error_vector(x);
  }

  gtsam::JacobianFactor::shared_ptr createL2Factor() const override {
    return factor_;
  }

  gtsam::JacobianFactor::shared_ptr createConstrainedFactor() const override;

  gtsam::MultiJacobian jacobian() const override;
};

/**
 * Inequality constraint base class.
 */
class InequalityConstraint {
protected:
  std::string name_;

public:
  typedef InequalityConstraint This;
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor. */
  InequalityConstraint(const std::string &name = "") : name_(name) {}

  /** Destructor. */
  virtual ~InequalityConstraint() {}

  const std::string &name() const { return name_; }

  std::string name_tmp() const {
    if (name_ == "") {
      return gtdynamics::GTDKeyFormatter(*keys().begin());
    }
    return name();
  }

  /**
   * @brief Check if constraint violation is within tolerance.
   *
   * @param x values to evalute constraint at.
   * @return bool representing if is feasible.
   */
  virtual bool feasible(const gtsam::Values &x) const = 0;

  /**
   * @brief Evaluate the constraint violation, g(x).
   *
   * @param x values to evalute constraint at.
   * @return a vector representing the constraint violation in each dimension.
   */
  virtual double operator()(const gtsam::Values &x) const = 0;

  virtual bool isActive(const gtsam::Values &x) const = 0;

  virtual double toleranceScaledViolation(const gtsam::Values &x) const = 0;

  /** @brief return the dimension of the constraint. */
  virtual size_t dim() const { return 1; };

  virtual gtsam::Vector tolerance() const = 0;

  /// Return keys of variables involved in the constraint.
  virtual std::set<gtsam::Key> keys() const { return std::set<gtsam::Key>(); }

  virtual EqualityConstraint::shared_ptr createEqualityConstraint() const = 0;

  virtual gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu = 1.0) const = 0;

  virtual gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu = 1.0) const = 0;

  virtual gtsam::NoiseModelFactor::shared_ptr
  createSmoothBarrierFactor(const double mu = 1.0,
                            const double buffer_width = 1.0) const = 0;

  virtual LinearInequalityConstraint::shared_ptr
  linearize(const gtsam::Values &values) const;

  virtual gtsam::MultiJacobian jacobians(const gtsam::Values &x) const = 0;

  virtual void print(const gtsam::KeyFormatter &key_formatter =
                         gtdynamics::GTDKeyFormatter) const;
};

/** Inequality constraint that force g(x) >= 0, where g(x) is a scalar-valued
 * function. */
class DoubleExpressionInequality : public InequalityConstraint {
public:
  typedef DoubleExpressionInequality This;
  typedef std::shared_ptr<This> shared_ptr;

protected:
  gtsam::Double_ expression_;
  double tolerance_;

public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param tolerance   scalar representing tolerance.
   */
  DoubleExpressionInequality(const gtsam::Double_ &expression,
                             const double &tolerance,
                             const std::string &name = "")
      : InequalityConstraint(name), expression_(expression),
        tolerance_(tolerance) {}

  // Inequality constraint g(x)>=0.
  static DoubleExpressionInequality::shared_ptr
  geq(const gtsam::Double_ &expression, const double &tolerance);

  // Inequality constraint g(x)<=0.
  static DoubleExpressionInequality::shared_ptr
  leq(const gtsam::Double_ &expression, const double &tolerance);

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values &x) const override;

  /** Evaluate the constraint function, g(x). */
  double operator()(const gtsam::Values &x) const override;

  double toleranceScaledViolation(const gtsam::Values &x) const override;

  bool isActive(const gtsam::Values &x) const override;

  std::set<gtsam::Key> keys() const override { return expression_.keys(); }

  EqualityConstraint::shared_ptr createEqualityConstraint() const override;

  gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu = 1.0) const override;

  gtsam::NoiseModelFactor::shared_ptr
  createSmoothBarrierFactor(const double mu = 1.0,
                            const double buffer_width = 1.0) const override;

  gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu = 1.0) const override;

  gtsam::MultiJacobian jacobians(const gtsam::Values &x) const override;

  gtsam::Vector tolerance() const override {
    return gtsam::Vector1(tolerance_);
  }

  const gtsam::Double_ &expression() const { return expression_; }
};

/** Inequality constraint that force g(x) >= 0, where g(x) is a scalar-valued
 * function. */
class TwinDoubleExpressionInequality : public InequalityConstraint {
public:
  typedef TwinDoubleExpressionInequality This;
  typedef std::shared_ptr<This> shared_ptr;

protected:
  DoubleExpressionInequality::shared_ptr ineq1_, ineq2_;
  gtsam::Vector2_ expression_;
  gtsam::Vector2 tolerance_;

public:
  /**
   * @brief Constructor.
   *
   * @param expression  expression representing g(x).
   * @param tolerance   scalar representing tolerance.
   */
  TwinDoubleExpressionInequality(
      const DoubleExpressionInequality::shared_ptr ineq1,
      const DoubleExpressionInequality::shared_ptr ineq2,
      const std::string &name = "")
      : InequalityConstraint(name == "" ? ineq1->name() : name), ineq1_(ineq1),
        ineq2_(ineq2), expression_(ConstructExpression(ineq1->expression(),
                                                       ineq2->expression())),
        tolerance_(ineq1->tolerance()(0), ineq2->tolerance()(0)) {}

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values &x) const override {
    return ineq1_->feasible(x) && ineq2_->feasible(x);
  }

  gtsam::Vector tolerance() const override { return tolerance_; }

  DoubleExpressionInequality::shared_ptr constraint1() const { return ineq1_; }

  DoubleExpressionInequality::shared_ptr constraint2() const { return ineq2_; }

  /** Evaluate the constraint function, g(x). */
  double operator()(const gtsam::Values &x) const override;

  double toleranceScaledViolation(const gtsam::Values &x) const override;

  bool isActive(const gtsam::Values &x) const override;

  size_t dim() const override;

  std::set<gtsam::Key> keys() const override;

  EqualityConstraint::shared_ptr createEqualityConstraint() const override;

  gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu = 1.0) const override;

  gtsam::NoiseModelFactor::shared_ptr
  createSmoothBarrierFactor(const double mu = 1.0,
                            const double buffer_width = 1.0) const override;

  gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu = 1.0) const override;

  gtsam::MultiJacobian jacobians(const gtsam::Values &x) const override;

  static gtsam::Vector2_ ConstructExpression(const gtsam::Double_ &expr1,
                                             const gtsam::Double_ &expr2);
};

class LinearInequalityConstraints
    : public std::vector<LinearInequalityConstraint::shared_ptr> {
private:
  using Base = std::vector<LinearInequalityConstraint::shared_ptr>;

  template <typename DERIVEDCONSTRAINT>
  using IsDerived =
      typename std::enable_if<std::is_base_of<LinearInequalityConstraint,
                                              DERIVEDCONSTRAINT>::value>::type;

public:
  typedef LinearInequalityConstraints This;
  typedef std::shared_ptr<This> shared_ptr;

  LinearInequalityConstraints() : Base() {}

  /// Emplace a shared pointer to constraint of given type.
  template <class DERIVEDCONSTRAINT, class... Args>
  IsDerived<DERIVEDCONSTRAINT> emplace_shared(Args &&...args) {
    push_back(std::allocate_shared<DERIVEDCONSTRAINT>(
        Eigen::aligned_allocator<DERIVEDCONSTRAINT>(),
        std::forward<Args>(args)...));
  }

  gtsam::GaussianFactorGraph
  constraintGraph(const gtsam::IndexSet &active_indices) const;

  void print(const gtsam::KeyFormatter &key_formatter =
                 gtdynamics::GTDKeyFormatter) const;
};

typedef std::map<size_t, LinearInequalityConstraint::shared_ptr>
    LinearIConstraintMap;

/// Container of InequalityConstraint.
class InequalityConstraints
    : public std::vector<InequalityConstraint::shared_ptr> {
private:
  using Base = std::vector<InequalityConstraint::shared_ptr>;

  template <typename DERIVEDCONSTRAINT>
  using IsDerived = typename std::enable_if<
      std::is_base_of<InequalityConstraint, DERIVEDCONSTRAINT>::value>::type;

public:
  typedef InequalityConstraints This;
  typedef std::shared_ptr<This> shared_ptr;

  InequalityConstraints() : Base() {}

  /// Add a set of equality constraints.
  void add(const InequalityConstraints &other) {
    insert(end(), other.begin(), other.end());
  }

  /// Emplace a shared pointer to constraint of given type.
  template <class DERIVEDCONSTRAINT, class... Args>
  IsDerived<DERIVEDCONSTRAINT> emplace_shared(Args &&...args) {
    push_back(std::allocate_shared<DERIVEDCONSTRAINT>(
        Eigen::aligned_allocator<DERIVEDCONSTRAINT>(),
        std::forward<Args>(args)...));
  }

  bool feasible(const gtsam::Values &x) const;

  /// Return keys involved in constraints.
  gtsam::KeySet keys() const;

  gtsam::VariableIndex varIndex() const;

  /// Return the total dimension of constraints.
  size_t dim() const;

  /// Evaluate the constraint violation (as L2 norm).
  double evaluateViolationL2Norm(const gtsam::Values &values) const;

  gtsam::NonlinearFactorGraph meritGraph(const double mu = 1.0,
                                         const bool smooth = false,
                                         const double buffer_width = 1.0) const;

  void print(const gtsam::KeyFormatter &key_formatter =
                 gtdynamics::GTDKeyFormatter) const;
};

} // namespace gtdynamics
