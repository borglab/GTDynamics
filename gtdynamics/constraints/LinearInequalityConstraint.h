/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  LinearInequalityConstraint.h
 * @brief Linear inequality constraints in constrained optimization.
 * @author: Yetong Zhang, Frank Dellaert
 */

#pragma once

#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/StdVector>
#include <map>
#include <memory>
#include <type_traits>
#include <vector>

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

  /** Destructor. */
  virtual ~LinearInequalityConstraint() {}

  /** @brief return the dimension of the constraint. */
  virtual size_t dim() const = 0;

  /** @brief Evaluate the constraint function. */
  virtual gtsam::Vector operator()(const gtsam::VectorValues &x) const = 0;

  /** @brief Check if constraint violation is within tolerance. */
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

using LinearIConstraintMap =
    std::map<size_t, LinearInequalityConstraint::shared_ptr>;

} // namespace gtdynamics
