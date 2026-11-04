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
  return linearization_->project(xi);
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
  return linearization_->blockingIndices(tangentVector);
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
std::shared_ptr<const IEManifoldLinearization>
IEConstraintManifold::constructLinearization(
    const NonlinearInequalityConstraints &i_constraints, const Values &values,
    const IndexSet &active_indices,
    const TangentSpaceBasis::shared_ptr &t_basis) {
  LinearIConstraintMap tangent, ambient;
  for (size_t index : active_indices) {
    const auto &constraint = i_constraints.at(index);
    const auto linear = LinearizedIConstraint(constraint, values);
    Matrix rows = Matrix::Zero(linear->rows(), t_basis->dim());
    std::vector<std::pair<Key, Matrix>> terms;
    for (auto it = linear->begin(); it != linear->end(); ++it) {
      rows += linear->getA(it) * t_basis->recoverJacobian(*it);
      terms.emplace_back(*it, linear->getA(it));
    }
    const auto noise = noiseModel::Diagonal::Sigmas(constraint->sigmas());
    const Vector zero = Vector::Zero(linear->rows());
    tangent.emplace(
        index, std::make_shared<JacobianLinearInequalityConstraint>(
                   std::make_shared<JacobianFactor>(1, rows, zero, noise)));
    ambient.emplace(index,
                    std::make_shared<JacobianLinearInequalityConstraint>(
                        std::make_shared<JacobianFactor>(terms, zero, noise)));
  }
  return std::make_shared<const IEManifoldLinearization>(tangent, ambient);
}

/* ************************************************************************* */
LinearIConstraintMap
IEConstraintManifold::linearActiveManifoldInequalityConstraints(
    const Key manifold_key) const {
  return linearization_->tangentConstraints(manifold_key);
}

/* ************************************************************************* */
LinearIConstraintMap
IEConstraintManifold::linearActiveBaseInequalityConstraints() const {
  return linearization_->ambientConstraints();
}

} // namespace gtdynamics
