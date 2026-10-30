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

#include <gtdynamics/cmcopt/TangentCone.h>
#include <gtsam/constrained/ActiveSetSolver.h>

#include <algorithm>

namespace gtdynamics {
using namespace gtsam;

/* ************************************************************************* */
std::pair<IndexSet, Vector> TangentCone::project(const Vector& xi) const {
  if (constraints_.size() == 0) {
    return {IndexSet(), xi};
  }

  const Key key = 1;
  QpProblem problem;
  problem.addCost(
      JacobianFactor(key, Matrix::Identity(xi.size(), xi.size()), xi));
  ActiveSetSolver::State workingSet;
  for (const auto& constraint : constraints_) {
    problem.addConstraint(
        LinearConstraint::LessEqual(constraint->createL2Factor()->whiten()));
    workingSet.activeInequalityRows.insert(
        workingSet.activeInequalityRows.end(), constraint->dim(), false);
  }
  Values initial;
  initial.insert(key, Vector::Zero(xi.size()).eval());
  auto params = std::make_shared<ActiveSetSolverParams>();
  // Constraints already use equality-reduced coordinates. Reuse the objective
  // factorization while solving each working set's dense Schur system.
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  params->regularization = 0.0;

  try {
    const auto [values, state] =
        ActiveSetSolver(problem, params).optimizeWithState(initial, workingSet);
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
    return {activeIndices, values.at<Vector>(key)};
  } catch (const IndeterminateSystemException& error) {
    std::cerr << "Tangent-cone projection failed: " << error.what() << '\n';
  } catch (const std::runtime_error& error) {
    std::cerr << "Tangent-cone projection failed: " << error.what() << '\n';
  }
  return {IndexSet(), Vector::Zero(xi.size())};
}

}  // namespace gtdynamics
