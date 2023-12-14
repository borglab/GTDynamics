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
IEConstraintManifold
IEConstraintManifold::retract(const Vector &xi,
                              const std::optional<IndexSet> &blocking_indices,
                              IERetractInfo *retract_info) const {
  auto tangent_vector = e_basis_->computeTangentVector(xi);
  return retract(tangent_vector, blocking_indices, retract_info);
}

/* ************************************************************************* */
IEConstraintManifold
IEConstraintManifold::retract(const VectorValues &delta,
                              const std::optional<IndexSet> &blocking_indices,
                              IERetractInfo *retract_info) const {
  return retractor_->retract(this, delta, blocking_indices, retract_info);
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
IEConstraintManifold::moveToBoundary(const IndexSet &active_indices,
                                     IERetractInfo *retract_info) const {
  return retractor_->moveToBoundary(this, active_indices, retract_info);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold() const {
  return ConstraintManifold(e_constraints_, values_, params_->ecm_params, false,
                            e_basis_);
}

/* ************************************************************************* */
ConstraintManifold IEConstraintManifold::eConstraintManifold(
    const IndexSet &active_indices) const {
  if (active_indices.size() == 0) {
    return eConstraintManifold();
  }

  // TODO: construct e-basis using createWithAdditionalConstraints
  gtdynamics::EqualityConstraints new_active_constraints;
  for (const auto &idx : active_indices) {
    new_active_constraints.emplace_back(
        i_constraints_->at(idx)->createEqualityConstraint());
  }
  auto active_constraints =
      std::make_shared<EqualityConstraints>(*e_constraints_);
  active_constraints->add(new_active_constraints);

  auto new_basis = e_basis_->createWithAdditionalConstraints(
      new_active_constraints, values_, params_->e_basis_build_from_scratch);
  return ConstraintManifold(active_constraints, values_, params_->ecm_params,
                            false, new_basis);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::IdentifyActiveConstraints(
    const gtdynamics::InequalityConstraints &i_constraints,
    const Values &values, const std::optional<IndexSet> &active_indices) {
  if (active_indices) {
    for (const auto& i: *active_indices) {
      if (!i_constraints.at(i)->isActive(values)) {
        std::cout << i_constraints.at(i)->name_tmp() << " is not active\t";
        std::cout << (*i_constraints.at(i))(values) << "\n";
      }
    }
    return *active_indices;
  }

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

  size_t num_rows = 0;
  for (const auto &active_idx : active_indices) {
    num_rows += i_constraints.at(active_idx)->dim();
  }
  std::vector<std::pair<size_t, size_t>> constraint_rows;
  Matrix A = Matrix::Zero(num_rows, t_basis->dim());
  size_t i = 0;
  for (const auto &active_idx : active_indices) {
    const auto &i_constraint = i_constraints.at(active_idx);
    size_t dim = i_constraint->dim();
    auto jacobians = i_constraint->jacobians(values);
    for (const Key &key : jacobians.keys()) {
      A.middleRows(i, dim) += jacobians.at(key) * t_basis->recoverJacobian(key);
    }
    constraint_rows.emplace_back(i, dim);
    i += dim;
  }

  auto cone = std::make_shared<TangentCone>(A, constraint_rows);
  return cone;
}

} // namespace gtsam
