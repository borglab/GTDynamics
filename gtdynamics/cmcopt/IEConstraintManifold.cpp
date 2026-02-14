#include <gtdynamics/cmcopt/IEConstraintManifold.h>

namespace gtdynamics {
using namespace gtsam;

namespace {

/**
 * Extract the block Jacobian d f / d x_key from a linearized factor.
 *
 * Why this exists:
 * 1) New constrained GTSAM APIs return linearized constraints as a generic
 *    `GaussianFactor::shared_ptr`.
 * 2) CMCOpt still needs per-key Jacobian blocks to project constraints into
 *    the manifold basis and to build `JacobianFactor`s for tangent-cone QP.
 * 3) The previous CMCOpt code path relied on custom inequality-constraint
 *    methods that directly returned jacobian maps; those methods no longer
 *    exist in the current GTSAM constraint classes.
 *
 * What it does:
 * - Walks the factor variable ordering to compute the starting column of `key`
 *   in the dense Jacobian matrix returned by `GaussianFactor::jacobian()`.
 * - Returns the middle block corresponding to that key's local dimension.
 *
 * Notes:
 * - This helper assumes `key` is present in `linear_factor`.
 * - It intentionally mirrors the existing helper used in
 *   `IPOptOptimizer.cpp` to keep behavior consistent across optimizers.
 */
Matrix RetrieveVarJacobian(const GaussianFactor::shared_ptr &linear_factor,
                           const Key &key) {
  size_t start_col = 0;
  size_t var_dim = 0;
  for (auto it = linear_factor->begin(); it != linear_factor->end(); it++) {
    if (*it == key) {
      var_dim = linear_factor->getDim(it);
      break;
    }
    start_col += linear_factor->getDim(it);
  }
  auto jacobian = linear_factor->jacobian().first;
  return jacobian.middleCols(start_col, var_dim);
}

/**
 * Linearize a nonlinear inequality constraint as a JacobianFactor.
 *
 * Why this exists:
 * - Current GTSAM inequality constraints expose `unwhitenedExpr(...)` and
 *   `createEqualityConstraint()`, but do not expose the old CMCOpt-style
 *   jacobian map API.
 * - CMCOpt needs a linearized constraint in matrix form to:
 *   1) detect blocking constraints from directional derivatives,
 *   2) project active constraints into manifold coordinates,
 *   3) build `LinearInequalityConstraint`s for tangent-cone projection.
 *
 * How it maps old behavior:
 * - We form the equality counterpart g(x)=0 of the inequality g(x)<=0 via
 *   `createEqualityConstraint()`.
 * - We linearize that equality at `values`.
 * - We normalize to `JacobianFactor` so downstream code can use block access
 *   (`rows()`, `getA(it)`, key iteration) without branching on factor type.
 *
 * This keeps the previous algorithmic structure intact while adapting to the
 * new constrained-GTSAM interface.
 */
JacobianFactor::shared_ptr LinearizedIConstraint(
    const NonlinearInequalityConstraint::shared_ptr &i_constraint,
    const Values &values) {
  auto linear_factor = i_constraint->createEqualityConstraint()->linearize(values);
  return std::make_shared<JacobianFactor>(*linear_factor);
}

}  // namespace


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
    const auto &i_constraint = i_constraints_->at(idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values_);
    Vector error = Vector::Zero(linear_factor->rows());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      if (tangent_vector.exists(*it)) {
        error += linear_factor->getA(it) * tangent_vector.at(*it);
      }
    }
    if ((error.array() < -1e-5).any()) {
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
  NonlinearEqualityConstraints new_active_constraints;
  for (const auto &idx : active_indices) {
    new_active_constraints.push_back(
        i_constraints_->at(idx)->createEqualityConstraint());
  }
  auto active_constraints =
      std::make_shared<NonlinearEqualityConstraints>(*e_constraints_);
  active_constraints->add(new_active_constraints);

  auto new_basis = e_basis_->createWithAdditionalConstraints(
      new_active_constraints, values_, params_->e_basis_build_from_scratch);
  return ConstraintManifold(active_constraints, values_, params_->ecm_params,
                            false, new_basis);
}

/* ************************************************************************* */
IndexSet IEConstraintManifold::IdentifyActiveConstraints(
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values, const std::optional<IndexSet> &active_indices) {
  if (active_indices) {
    for (const auto &i : *active_indices) {
      if (!i_constraints.at(i)->active(values)) {
        std::cout << "inequality constraint " << i << " is not active\t";
        std::cout << i_constraints.at(i)->unwhitenedExpr(values).transpose()
                  << "\n";
      }
    }
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
TangentCone::shared_ptr IEConstraintManifold::ConstructTangentCone(
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values, const IndexSet &active_indices,
    const TspaceBasis::shared_ptr &t_basis) {

  LinearInequalityConstraints constraints;
  for (const auto &constraint_idx : active_indices) {
    auto i_constraint = i_constraints.at(constraint_idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values);
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
IEConstraintManifold::linearActiveManIConstraints(
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
IEConstraintManifold::linearActiveBaseIConstraints() const {
  LinearIConstraintMap active_constraints;

  for (const auto &constraint_idx : active_indices_) {
    auto i_constraint = i_constraints_->at(constraint_idx);
    auto linear_factor = LinearizedIConstraint(i_constraint, values_);
    std::vector<std::pair<Key, Matrix>> jacobians;
    jacobians.reserve(linear_factor->size());
    for (auto it = linear_factor->begin(); it != linear_factor->end(); ++it) {
      jacobians.emplace_back(*it, RetrieveVarJacobian(linear_factor, *it));
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
