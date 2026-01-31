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
double
ComputeAuxiliaryValue(DoubleExpressionInequality::shared_ptr i_constraint,
                      const Values &values) {
  double constraint_violation = (*i_constraint)(values);
  if (constraint_violation < 0) {
    return 0;
  }
  double aux_val = sqrt(constraint_violation);
  return aux_val;
}

/* ************************************************************************* */
EqualityConstraint::shared_ptr
CreateAuxiliaryConstraint(DoubleExpressionInequality::shared_ptr i_constraint,
                          Key aux_key) {
  auto expr = i_constraint->expression();
  Double_ aux_expr(aux_key);
  Double_ new_expr = expr - aux_expr * aux_expr;
  double tolerance = i_constraint->tolerance()(0);
  return std::make_shared<DoubleExpressionEquality>(new_expr, tolerance);
}

/* ************************************************************************* */
EConsOptProblem IEConsOptProblem::auxiliaryProblem() const {
  EqualityConstraints aux_constraints;
  Values aux_values;

  uint64_t k = 0;
  for (const auto &i_constraint : iConstraints()) {
    if (DoubleExpressionInequality::shared_ptr p =
            std::dynamic_pointer_cast<DoubleExpressionInequality>(
                i_constraint)) {
      Key aux_key = AuxilaryKey(k++);
      aux_constraints.emplace_back(CreateAuxiliaryConstraint(p, aux_key));
      aux_values.insert(aux_key, ComputeAuxiliaryValue(p, values_));
    } else if (TwinDoubleExpressionInequality::shared_ptr p =
                   std::dynamic_pointer_cast<TwinDoubleExpressionInequality>(
                       i_constraint)) {
      DoubleExpressionInequality::shared_ptr p1 = p->constraint1();
      Key aux_key1 = AuxilaryKey(k++);
      aux_constraints.emplace_back(CreateAuxiliaryConstraint(p1, aux_key1));
      aux_values.insert(aux_key1, ComputeAuxiliaryValue(p1, values_));
      DoubleExpressionInequality::shared_ptr p2 = p->constraint1();
      Key aux_key2 = AuxilaryKey(k++);
      aux_constraints.emplace_back(CreateAuxiliaryConstraint(p2, aux_key2));
      aux_values.insert(aux_key2, ComputeAuxiliaryValue(p2, values_));
    }
  }

  EqualityConstraints all_constraints = eConstraints();
  all_constraints.insert(all_constraints.end(), aux_constraints.begin(),
                         aux_constraints.end());
  Values all_values = values_;
  all_values.insert(aux_values);

  return EConsOptProblem(costs(), all_constraints, all_values);
}

} // namespace gtdynamics
