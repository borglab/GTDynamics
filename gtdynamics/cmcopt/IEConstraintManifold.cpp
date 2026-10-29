/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IEConstraintManifold.cpp
 * @brief Implement the equality and inequality constraint manifold.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IEConstraintManifold.h>

namespace gtdynamics {
using namespace gtsam;

namespace {

/** Form Dg(x) and -g(x) without whitening or clipping feasible rows.
 *
 * Equality-factor linearization already whitens by the constraint sigma.
 * Callers attach that sigma to the resulting linear inequality, so using an
 * equality linearization here would apply the scaling twice. Read g and Dg
 * directly; unlike an inequality's error Jacobian, Dg also retains inward rows.
 */
JacobianFactor::shared_ptr LinearizedIConstraint(
    const NonlinearInequalityConstraint::shared_ptr &constraint,
    const Values &values) {
  std::vector<Matrix> jacobians(constraint->size());
  const Vector expression = constraint->unwhitenedExpr(values, &jacobians);
  std::vector<std::pair<Key, Matrix>> terms;
  terms.reserve(constraint->size());
  for (size_t index = 0; index < constraint->size(); ++index) {
    terms.emplace_back(constraint->keys()[index], jacobians[index]);
  }
  return std::make_shared<JacobianFactor>(terms, -expression);
}

}  // namespace


TangentSpaceMetric::shared_ptr IEConstraintManifold::projectionMetric(
    const IERetractorParams &params,
    const std::optional<KeyVector> &priorKeys) const {
  const auto precisions = params.priorPrecisions(
      values_, priorKeys ? *priorKeys : values_.keys());
  std::lock_guard<std::mutex> lock(metric_cache_->mutex);
  if (!metric_cache_->metric || !metric_cache_->metric->matches(precisions, params.whitenAffineCoordinates))
    metric_cache_->metric =
        std::make_shared<TangentSpaceMetric>(*e_basis_, values_, precisions, params.whitenAffineCoordinates);
  return metric_cache_->metric;
}

/* ************************************************************************* */
std::pair<IndexSet, Vector>
IEConstraintManifold::projectTangentCone(const Vector &xi) const {
  auto result = i_cone_->project(xi);

  // TangentCone::project returns row indices in the active-constraint list;
  // map them back to the original inequality indices from thesis Eq. (4.3).
  IndexSet blocking_indices;
  std::vector<size_t> active_indices_vector(active_indices_.begin(),
                                            active_indices_.end());
  for (const auto &index : result.first) {
    blocking_indices.insert(active_indices_vector.at(index));
  }
  return std::make_pair(blocking_indices, result.second);
}

/* ************************************************************************* */
std::pair<IndexSet, VectorValues> IEConstraintManifold::projectTangentCone(
    const VectorValues &tangentVector) const {
  Vector xi = e_basis_->computeXi(tangentVector);
  auto result = projectTangentCone(xi);
  VectorValues projected_tangent_vector =
      e_basis_->computeTangentVector(result.second);
  return std::make_pair(result.first, projected_tangent_vector);
}

/* ************************************************************************* */
IEConstraintManifold
IEConstraintManifold::retract(const Vector &xi,
                              const std::optional<IndexSet> &blocking_indices,
                              IERetractionInfo *retract_info) const {
  auto tangentVector = e_basis_->computeTangentVector(xi);
  return retract(tangentVector, blocking_indices, retract_info);
}

/* ************************************************************************* */
IEConstraintManifold
IEConstraintManifold::retract(const VectorValues &delta,
                              const std::optional<IndexSet> &blocking_indices,
                              IERetractionInfo *retract_info) const {
  return retractor_->retract(this, delta, blocking_indices, retract_info);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::blockingIndices(
    const VectorValues &tangentVector) const {
  IndexSet blocking_indices;

  for (const auto &idx : active_indices_) {
    const auto &i_constraint = i_constraints_->at(idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values_);
    Vector error = Vector::Zero(linear_factor->rows());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      if (tangentVector.exists(*it)) {
        error += linear_factor->getA(it) * tangentVector.at(*it);
      }
    }
    // A positive directional derivative means this g(x)<=0 face would be
    // violated by the proposed tangent step; these are blocking constraints
    // in the sense of thesis Eq. (4.45).
    if ((error.array() > 1e-5).any()) {
      blocking_indices.insert(idx);
    }
  }
  return blocking_indices;
}

/* ************************************************************************* */
IEConstraintManifold
IEConstraintManifold::moveToBoundary(const IndexSet &active_indices,
                                     IERetractionInfo *retract_info) const {
  return retractor_->moveToBoundary(this, active_indices, retract_info);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold() const {
  return ConstraintManifold(e_constraints_, values_, params_->equalityManifoldParams, false,
                            e_basis_);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold(
    const IndexSet &active_indices) const {
  if (active_indices.size() == 0) {
    return eConstraintManifold();
  }

  // TODO: construct e-basis using createWithAdditionalConstraints
  NonlinearEqualityConstraints new_active_constraints;
  for (const auto &idx : active_indices) {
    new_active_constraints.push_back(
        i_constraints_->at(idx)->createEqualityConstraint());
  }
  auto active_constraints =
      std::make_shared<NonlinearEqualityConstraints>(*e_constraints_);
  active_constraints->add(new_active_constraints);

  auto new_basis = e_basis_->createWithAdditionalConstraints(
      new_active_constraints, values_, params_->equalityBasisBuildFromScratch);
  return ConstraintManifold(active_constraints, values_, params_->equalityManifoldParams,
                            false, new_basis);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::identifyActiveConstraints(
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values, const std::optional<IndexSet> &active_indices) {
  if (active_indices) {
    // A retractor can approach a boundary from inside its feasibility
    // tolerance. Its explicit mode is authoritative even if g is slightly
    // negative; active(), which tests g>=0 exactly, is not a validity check.
    return *active_indices;
  }

  IndexSet active_set;
  for (size_t i = 0; i < i_constraints.size(); i++) {
    if (i_constraints.at(i)->active(values)) {
      active_set.insert(i);
    }
  }
  return active_set;
}

/* ************************************************************************* */
TangentCone::shared_ptr IEConstraintManifold::constructTangentCone(
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values, const IndexSet &active_indices,
    const TangentSpaceBasis::shared_ptr &t_basis) {

  LinearInequalityConstraints constraints;
  for (const auto &constraint_idx : active_indices) {
    auto i_constraint = i_constraints.at(constraint_idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values);
    // Form Dg_A(x) B_x, i.e. the K matrix in thesis Eq. (4.15), so cone
    // projection happens in equality-manifold coordinates.
    Matrix man_jacobian = Matrix::Zero(linear_factor->rows(), t_basis->dim());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      const Key key = *it;
      man_jacobian += linear_factor->getA(it) * t_basis->recoverJacobian(key);
    }
    auto noise_model = noiseModel::Diagonal::Sigmas(i_constraint->sigmas());
    auto jacobian_factor = std::make_shared<JacobianFactor>(
        1, man_jacobian, Vector::Zero(linear_factor->rows()),
        noise_model);
    constraints.emplace_shared<JacobianLinearInequalityConstraint>(
            jacobian_factor);
  }

  auto cone = std::make_shared<TangentCone>(constraints);
  return cone;
}

/* ************************************************************************* */
LinearIConstraintMap
IEConstraintManifold::linearActiveManifoldInequalityConstraints(
    const Key manifold_key) const {

  LinearIConstraintMap active_constraints;

  for (const auto &constraint_idx : active_indices_) {
    auto i_constraint = i_constraints_->at(constraint_idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values_);
    Matrix man_jacobian = Matrix::Zero(linear_factor->rows(), dim());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      const Key key = *it;
      man_jacobian += linear_factor->getA(it) * e_basis_->recoverJacobian(key);
    }
    auto noise_model = noiseModel::Diagonal::Sigmas(i_constraint->sigmas());
    auto jacobian_factor = std::make_shared<JacobianFactor>(
        manifold_key, man_jacobian, Vector::Zero(linear_factor->rows()),
        noise_model);
    auto linear_constraint =
        std::make_shared<JacobianLinearInequalityConstraint>(
            jacobian_factor);
    active_constraints.insert({constraint_idx, linear_constraint});
  }

  return active_constraints;
}

/* ************************************************************************* */
LinearIConstraintMap
IEConstraintManifold::linearActiveBaseInequalityConstraints() const {
  LinearIConstraintMap active_constraints;

  for (const auto &constraint_idx : active_indices_) {
    auto i_constraint = i_constraints_->at(constraint_idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values_);
    std::vector<std::pair<Key, Matrix>> jacobians;
    jacobians.reserve(linear_factor->size());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      jacobians.emplace_back(*it, linear_factor->getA(it));
    }
    auto noise_model = noiseModel::Diagonal::Sigmas(i_constraint->sigmas());
    auto jacobian_factor = std::make_shared<JacobianFactor>(
        jacobians, Vector::Zero(linear_factor->rows()), noise_model);
    auto linear_constraint =
        std::make_shared<JacobianLinearInequalityConstraint>(
            jacobian_factor);
    active_constraints.insert({constraint_idx, linear_constraint});
  }

  return active_constraints;
}

} // namespace gtdynamics
