/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptProblem.h
 * @brief Constrained optimization problems.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/constraints/EqualityConstraint.h>
#include <gtdynamics/constraints/InequalityConstraint.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/** Equality-constrained optimization problem, in the form of
 * argmin_x 0.5||f(X)||^2
 * s.t.     h(X) = 0
 * where X represents the variables, 0.5||f(X)||^2 represents the quadratic cost
 * functions, h(X)=0 represents the constraints.
 */
struct EConsOptProblem {
  typedef std::function<void(const Values &values)> EvalFunc;

  NonlinearFactorGraph costs_;        // cost function, ||f(X)||^2
  EqualityConstraints e_constraints_; // equality constraints. h(X)=0
  Values values_;                     // values of all variables, X
  EvalFunc eval_func;
  /// Constructor.
  EConsOptProblem(const NonlinearFactorGraph &costs,
                  const EqualityConstraints &constraints, const Values &values)
      : costs_(costs), e_constraints_(constraints), values_(values) {}

  const NonlinearFactorGraph &costs() const { return costs_; }
  const EqualityConstraints &constraints() const { return e_constraints_; }
  const Values &initValues() const { return values_; }

  double evaluateCost(const Values &values) const {
    return costs_.error(values);
  }

  double evaluateEConstraintViolationL2Norm(const Values &values) const {
    return e_constraints_.evaluateViolationL2Norm(values);
  }

  /// Evaluate the dimension of costs.
  size_t costsDimension() const;

  /// Evaluate the dimension of constriants.
  size_t constraintsDimension() const { return e_constraints_.dim(); }

  /// Evaluate the dimension of variables.
  size_t valuesDimension() const { return values_.dim(); }
};

/** Equality-Inequality-constrained optimization problem, in the form of
 * argmin_x 0.5||f(X)||^2
 * s.t.     h(X) = 0
 *          g(X) >= 0
 * where X represents the variables, 0.5||f(X)||^2 represents the quadratic cost
 * functions, h(X)=0 represents the constraints.
 */
struct IEConsOptProblem : public EConsOptProblem {
  InequalityConstraints i_constraints_;

  /// Constructor.
  IEConsOptProblem(const NonlinearFactorGraph &costs,
                   const EqualityConstraints &e_constraints,
                   const InequalityConstraints &i_constraints,
                   const Values &values)
      : EConsOptProblem(costs, e_constraints, values),
        i_constraints_(i_constraints) {}

  const EqualityConstraints &eConstraints() const { return constraints(); }
  const InequalityConstraints &iConstraints() const { return i_constraints_; }

  double evaluateIConstraintViolationL2Norm(const Values &values) const {
    return i_constraints_.evaluateViolationL2Norm(values);
  }

  /// Equivalent equality-constrained optimization probelm with auxiliary
  /// variables z. Inequality constraints g(x)>=0 are transformed into equality
  /// constraints g(x)-z^2=0.
  EConsOptProblem auxiliaryProblem() const;
};

} // namespace gtsam
