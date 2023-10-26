#include <gtdynamics/imanifold/IEConstraintManifold.h>

namespace gtsam {

/* ************************************************************************* */
std::pair<IndexSet, Vector>
IEConstraintManifold::projectTangentCone(const Vector &xi) const {
  auto result = i_cone_->project(xi);

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
    const VectorValues &tangent_vector) const {
  Vector xi = e_basis_->computeXi(tangent_vector);
  auto result = projectTangentCone(xi);
  VectorValues projected_tangent_vector =
      e_basis_->computeTangentVector(result.second);
  return std::make_pair(result.first, projected_tangent_vector);
}

/* ************************************************************************* */
IEConstraintManifold IEConstraintManifold::retract(
    const Vector &xi, const std::optional<IndexSet> &blocking_indices) const {
  auto tangent_vector = e_basis_->computeTangentVector(xi);
  return retract(tangent_vector, blocking_indices);
}

/* ************************************************************************* */
IEConstraintManifold IEConstraintManifold::retract(
    const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {
  return retractor_->retract(this, delta, blocking_indices);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::blockingIndices(
    const VectorValues &tangent_vector) const {
  IndexSet blocking_indices;

  for (const auto &idx : active_indices_) {
    // TODO: store the jacobians to avoid recomputation
    const auto &i_constraint = i_constraints_->at(idx);
    auto jacobians = i_constraint->jacobians(values_);
    double error = 0;
    for (const auto &it : jacobians) {
      const Key &key = it.first;
      const Matrix &jac = it.second;
      error += (jac * tangent_vector.at(key))(0);
    }
    if (error < -1e-5) {
      blocking_indices.insert(idx);
    }
  }
  return blocking_indices;
}

/* ************************************************************************* */
IEConstraintManifold
IEConstraintManifold::moveToBoundary(const IndexSet &active_indices) const {
  return retractor_->moveToBoundary(this, active_indices);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold() const {
  return ConstraintManifold(e_cc_, values_, params_->ecm_params, false,
                            e_basis_);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold(
    const IndexSet &active_indices) const {
  // TODO: construct e-basis using createWithAdditionalConstraints
  gtdynamics::EqualityConstraints active_constraints = e_cc_->constraints_;
  gtdynamics::EqualityConstraints new_active_constraints;
  KeySet unconstrained_keys = e_cc_->unconstrained_keys_;
  for (const auto &idx : active_indices) {
    for (const Key &key : i_constraints_->at(idx)->keys()) {
      if (unconstrained_keys.exists(key)) {
        unconstrained_keys.erase(key);
      }
    }
    new_active_constraints.emplace_back(
        i_constraints_->at(idx)->createEqualityConstraint());
  }
  active_constraints.add(new_active_constraints);
  auto new_cc = std::make_shared<gtsam::ConnectedComponent>(active_constraints,
                                                            unconstrained_keys);
  if (params_->ecm_params->basis_params->use_basis_keys &&
      new_active_constraints.size() > 0) {

    // update basis keys
    size_t new_manifold_dim = dim_ - new_active_constraints.dim();
    auto basis_keys = params_->ecm_params->basis_key_func(e_cc_);
    auto new_constrained_keys = new_active_constraints.keys();
    // TODO: this only works for simple ineq constraint cases
    KeyVector new_basis_keys;
    for (const Key& key: basis_keys) {
      if (!new_constrained_keys.exists(key)) {
        new_basis_keys.push_back(key);
      }
    }

    // create basis here
    auto new_basis =
        TspaceBasis::create(params_->ecm_params->basis_params, new_cc, values_,
                            new_basis_keys, new_manifold_dim);
    return ConstraintManifold(new_cc, values_, params_->ecm_params, false, new_basis);
  }
  return ConstraintManifold(new_cc, values_, params_->ecm_params, false);
  // auto new_basis = e_basis_->createWithAdditionalConstraints(active_constraints);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::IdentifyActiveConstraints(
    const gtdynamics::InequalityConstraints &i_constraints,
    const Values &values) {
  IndexSet active_set;
  for (size_t i = 0; i < i_constraints.size(); i++) {
    if (i_constraints.at(i)->isActive(values)) {
      active_set.insert(i);
    }
  }
  return active_set;
}

/* ************************************************************************* */
TangentCone::shared_ptr IEConstraintManifold::ConstructTangentCone(
    const gtdynamics::InequalityConstraints &i_constraints,
    const Values &values, const IndexSet &active_indices,
    const TspaceBasis::shared_ptr &t_basis) {
  Matrix A = Matrix::Zero(active_indices.size(), t_basis->dim());
  std::vector<size_t> active_indices_vec(active_indices.begin(),
                                         active_indices.end());

  for (size_t i = 0; i < active_indices_vec.size(); i++) {
    const auto &i_constraint = i_constraints.at(active_indices_vec[i]);
    auto jacobians = i_constraint->jacobians(values);

    for (const Key &key : jacobians.keys()) {
      A.row(i) += jacobians.at(key) * t_basis->recoverJacobian(key);
    }
  }

  auto cone = std::make_shared<TangentCone>(A);
  return cone;
}

} // namespace gtsam
