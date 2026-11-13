/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TangentCone.cpp
 * @brief Tangent cone implementations.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/RobustQp.h>
#include <gtdynamics/constrained/TangentCone.h>
#include <gtsam/constrained/ActiveSetSolver.h>

#include <algorithm>

namespace gtdynamics {
using namespace gtsam;

/* ************************************************************************* */
std::pair<IndexSet, Vector> TangentCone::project(const Vector& xi) const {
  if (constraints_.size() == 0) {
    return {IndexSet(), xi};
  }

  // Uniform coordinate scaling preserves this Euclidean projection, including
  // affine bounds. Large trajectory gradients otherwise make the AL fallback's
  // absolute stationarity test unnecessarily ill-conditioned.
  const double scale = std::max(1.0, xi.norm());
  const Key key = 1;
  QpProblem problem;
  problem.addCost(
      JacobianFactor(key, Matrix::Identity(xi.size(), xi.size()), xi / scale));
  ActiveSetSolver::State workingSet;
  for (const auto& constraint : constraints_) {
    const auto original = constraint->createL2Factor()->whiten();
    Matrix A = original.getA(original.begin());
    Vector b = original.getb() / scale;
    // Positive row scaling leaves each half-space unchanged.
    for (size_t row = 0; row < A.rows(); ++row) {
      const double norm = A.row(row).norm();
      if (norm > 0) {
        A.row(row) /= norm;
        b(row) /= norm;
      }
    }
    problem.addConstraint(
        LinearConstraint::LessEqual(JacobianFactor(key, A, b)));
    workingSet.activeInequalityRows.insert(
        workingSet.activeInequalityRows.end(), constraint->dim(), false);
  }
  Values initial;
  initial.insert(key, Vector::Zero(xi.size()).eval());
  try {
    const auto state = internal::robustQp(problem, initial, workingSet, 1000);
    if (!state.converged)
      throw std::runtime_error("Tangent-cone QP iteration budget exhausted");
    IndexSet activeIndices;
    size_t offset = 0;
    for (size_t index = 0; index < constraints_.size(); ++index) {
      const size_t dim = constraints_[index]->dim();
      // CMCOpt can impose a whole constraint as an equality only if all its
      // rows are active. GTSAM handles partially active blocks internally.
      if (dim > 0 &&
          std::all_of(state.activeInequalityRows.begin() + offset,
                      state.activeInequalityRows.begin() + offset + dim,
                      [](bool active) { return active; })) {
        activeIndices.insert(index);
      }
      offset += dim;
    }
    return {activeIndices, scale * state.values.at<Vector>(key)};
  } catch (const IndeterminateSystemException& error) {
    throw std::runtime_error(std::string("Tangent-cone projection failed: ") +
                             error.what());
  } catch (const std::runtime_error& error) {
    throw std::runtime_error(std::string("Tangent-cone projection failed: ") +
                             error.what());
  }
}

}  // namespace gtdynamics
