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

#include <gtdynamics/constrained_optimizer/ConstrainedOptProblem.h>
#include <gtdynamics/utils/DynamicsSymbol.h>

namespace gtdynamics {

using gtsam::Double_;
using gtsam::Key;
using gtsam::ScalarExpressionInequalityConstraint;
using gtsam::Values;

/* ************************************************************************* */
size_t EConsOptProblem::costsDimension() const {
  size_t costs_dim = 0;
  for (const auto &factor : costs_) {
    costs_dim += factor->dim();
  }
  return costs_dim;
}

/* ************************************************************************* */
Key AuxilaryKey(uint64_t k) {
  return gtdynamics::DynamicsSymbol::SimpleSymbol("ax", k);
}

/* ************************************************************************* */
double ComputeAuxiliaryValue(
    const ScalarExpressionInequalityConstraint::shared_ptr &i_constraint,
    const Values &values) {
  double constraint_value = i_constraint->expression().value(values);
  if (constraint_value > 0) {
    return 0;
  }
  return sqrt(-constraint_value);
}

/* ************************************************************************* */
gtsam::NonlinearEqualityConstraint::shared_ptr CreateAuxiliaryConstraint(
    const ScalarExpressionInequalityConstraint::shared_ptr &i_constraint,
    Key aux_key) {
  auto expr = i_constraint->expression();
  Double_ aux_expr(aux_key);
  Double_ new_expr = expr + aux_expr * aux_expr;
  double tolerance = i_constraint->sigmas()(0);
  return std::make_shared<gtsam::ExpressionEqualityConstraint<double>>(
      new_expr, 0.0, gtsam::Vector1(tolerance));
}

/* ************************************************************************* */
EConsOptProblem IEConsOptProblem::auxiliaryProblem() const {
  gtsam::NonlinearEqualityConstraints aux_constraints;
  Values aux_values;

  uint64_t k = 0;
  for (const auto &i_constraint : iConstraints()) {
    auto scalar_constraint =
        std::dynamic_pointer_cast<ScalarExpressionInequalityConstraint>(
            i_constraint);
    if (!scalar_constraint) {
      throw std::runtime_error(
          "auxiliaryProblem only supports ScalarExpressionInequalityConstraint");
    }
    Key aux_key = AuxilaryKey(k++);
    aux_constraints.push_back(CreateAuxiliaryConstraint(scalar_constraint,
                                                        aux_key));
    aux_values.insert(aux_key,
                      ComputeAuxiliaryValue(scalar_constraint, values_));
  }

  gtsam::NonlinearEqualityConstraints all_constraints = eConstraints();
  for (const auto& constraint : aux_constraints) {
    all_constraints.push_back(constraint);
  }
  Values all_values = values_;
  all_values.insert(aux_values);

  return EConsOptProblem(costs(), all_constraints, all_values);
}

} // namespace gtdynamics
