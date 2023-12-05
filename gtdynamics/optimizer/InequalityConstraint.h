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
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtdynamics {

/**
 * Equality constraint base class.
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

  const std::string &name() { return name_; }

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

  /// Return keys of variables involved in the constraint.
  virtual std::set<gtsam::Key> keys() const { return std::set<gtsam::Key>(); }

  virtual EqualityConstraint::shared_ptr createEqualityConstraint() const = 0;

  virtual gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu) const = 0;

  virtual gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu) const = 0;

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
  geq(const gtsam::Double_ &expression, const double &tolerance) {
    return std::make_shared<DoubleExpressionInequality>(expression, tolerance);
  }

  // Inequality constraint g(x)<=0.
  static DoubleExpressionInequality::shared_ptr
  leq(const gtsam::Double_ &expression, const double &tolerance) {
    gtsam::Double_ neg_expr = gtsam::Double_(0.0) - expression;
    return std::make_shared<DoubleExpressionInequality>(neg_expr, tolerance);
  }

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values &x) const override {
    return expression_.value(x) >= 0;
  }

  /** Evaluate the constraint function, g(x). */
  double operator()(const gtsam::Values &x) const override {
    return expression_.value(x);
  }

  double toleranceScaledViolation(const gtsam::Values &x) const override {
    double error = expression_.value(x);
    if (error >= 0) {
      return 0;
    } else {
      return -error / tolerance_;
    }
  }

  bool isActive(const gtsam::Values &x) const override {
    double error = expression_.value(x);
    return abs(error / tolerance_) < 1e-5;
  }

  std::set<gtsam::Key> keys() const override { return expression_.keys(); }

  EqualityConstraint::shared_ptr createEqualityConstraint() const override {
    return std::make_shared<DoubleExpressionEquality>(expression_, tolerance_);
  }

  gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu) const override {
    return std::make_shared<gtsam::BarrierFactor>(createL2Factor(mu), true);
  }

  gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu) const override {
    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, tolerance_ / sqrt(mu));
    return std::make_shared<gtsam::ExpressionFactor<double>>(noise, 0.0,
                                                             expression_);
  }

  gtsam::MultiJacobian jacobians(const gtsam::Values &x) const override;

  double tolerance() const { return tolerance_; }

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
      : InequalityConstraint(name), ineq1_(ineq1), ineq2_(ineq2),
        expression_(
            ConstructExpression(ineq1->expression(), ineq2->expression())),
        tolerance_(ineq1->tolerance(), ineq2->tolerance()) {}

  /** Check if constraint violation is within tolerance. */
  bool feasible(const gtsam::Values &x) const override {
    return ineq1_->feasible(x) && ineq2_->feasible(x);
  }

  DoubleExpressionInequality::shared_ptr constraint1() const {return ineq1_; }

  DoubleExpressionInequality::shared_ptr constraint2() const {return ineq2_; }

  /** Evaluate the constraint function, g(x). */
  double operator()(const gtsam::Values &x) const override {
    // TODO: this should return a vector
    double eval1 = (*ineq1_)(x);
    double eval2 = (*ineq2_)(x);
    return sqrt(eval1 * eval1 + eval2 * eval2);
  }

  double toleranceScaledViolation(const gtsam::Values &x) const override {
    double error1 = ineq1_->toleranceScaledViolation(x);
    double error2 = ineq2_->toleranceScaledViolation(x);
    return sqrt(error1 * error1 + error2 * error2);
  }

  bool isActive(const gtsam::Values &x) const override {
    return ineq1_->isActive(x) || ineq2_->isActive(x);
  }

  size_t dim() const override { return 2; };

  std::set<gtsam::Key> keys() const override {
    std::set<gtsam::Key> all_keys = ineq1_->keys();
    all_keys.merge(ineq2_->keys());
    return all_keys;
  }

  EqualityConstraint::shared_ptr createEqualityConstraint() const override {
    return std::make_shared<VectorExpressionEquality<2>>(expression_,
                                                         tolerance_);
  }

  gtsam::NoiseModelFactor::shared_ptr
  createBarrierFactor(const double mu) const override {
    return std::make_shared<gtsam::BarrierFactor>(createL2Factor(mu), true);
  }

  gtsam::NoiseModelFactor::shared_ptr
  createL2Factor(const double mu) const override {
    auto noise =
        gtsam::noiseModel::Isotropic::Sigmas(1 / sqrt(mu) * tolerance_);
    return std::make_shared<gtsam::ExpressionFactor<gtsam::Vector2>>(
        noise, gtsam::Vector2::Zero(), expression_);
  }

  gtsam::MultiJacobian jacobians(const gtsam::Values &x) const override {
    auto jac1 = ineq1_->jacobians(x);
    auto jac2 = ineq2_->jacobians(x);
    // gtsam::PrintKeySet(ineq1_->keys(), "ineq1", GTDKeyFormatter);
    // gtsam::PrintKeySet(ineq2_->keys(), "ineq2", GTDKeyFormatter);
    // jac1.print("jac1", GTDKeyFormatter);
    // jac2.print("jac2", GTDKeyFormatter);
    return gtsam::MultiJacobian::VerticalStack(jac1, jac2);
  }

  static gtsam::Vector2_ ConstructExpression(const gtsam::Double_ &expr1,
                                             const gtsam::Double_ &expr2);
};

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

  gtsam::NonlinearFactorGraph meritGraph(const double mu = 1.0) const;
};

} // namespace gtdynamics

namespace gtsam {

class IndexSet : public std::set<size_t> {
public:
  using base = std::set<size_t>;
  using base::base;

  bool exists(const size_t idx) const { return find(idx) != end(); }

  void print(const std::string &s = "") const {
    std::cout << s;
    for (const auto &val : *this) {
      std::cout << val << "\t";
    }
    std::cout << std::endl;
  }
};

struct IndexSetMap : public std::map<Key, IndexSet> {
public:
  bool exists(const Key &key) const { return find(key) != end(); }

  void addIndices(const Key &key, const IndexSet &index_set) {
    if (!exists(key)) {
      insert({key, index_set});
    } else {
      IndexSet &current_indices = at(key);
      current_indices.insert(index_set.begin(), index_set.end());
    }
  }

  void mergeWith(const IndexSetMap &new_map) {
    for (const auto &it : new_map) {
      addIndices(it.first, it.second);
    }
  }
};

} // namespace gtsam
