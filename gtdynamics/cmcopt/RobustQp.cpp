/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtdynamics/cmcopt/RobustQp.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtdynamics {
namespace internal {
gtsam::ActiveSetSolverState robustQp(
    const gtsam::QpProblem& problem, const gtsam::Values& initial,
    const gtsam::ActiveSetSolver::State& workingSet, size_t maxIterations) {
  using namespace gtsam;
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->regularization = 0;
  params->maxIterations = maxIterations;
  for (auto backend : {ActiveSetSolverParams::QpSubproblemSolver::Dense,
                       ActiveSetSolverParams::QpSubproblemSolver::Sparse}) {
    params->qpSubproblemSolver = backend;
    try {
      const auto [values, state] = ActiveSetSolver(problem, params)
                                       .optimizeWithState(initial, workingSet);
      // Exhausting the caller's iteration budget is an explicit failure, not
      // permission to spend another budget in a different solver.
      return state;
    } catch (const IndeterminateSystemException&) {
      // A dependent active set can leave its dual system underdetermined.
    } catch (const std::runtime_error&) {
      // The same cone can have dependent active faces at an edge or apex.
    } catch (const std::invalid_argument&) {
      // AL can also recover a trial with sub-tolerance primal roundoff.
    }
  }

  auto al = std::make_shared<AugmentedLagrangianParams>();
  al->maxIterations = std::min<size_t>(80, maxIterations);
  al->absoluteViolationTolerance = 1e-9;
  al->absoluteStationarityTolerance = 1e-7;
  al->bclBetaOmega = .1;
  al->lmParams.maxIterations = 200;
  al->lmParams.absoluteErrorTol = al->lmParams.relativeErrorTol = 0;
  al->lmParams.lambdaUpperBound = 1e12;
  // CMC inequality blocks may have multiple rows, whereas native AL consumes
  // scalar inequalities. Split their exact affine expressions without changing
  // whitening, row order, or the QP's direct vector coordinate convention.
  NonlinearInequalityConstraints scalarConstraints;
  for (const auto& constraint : problem.iConstraints()) {
    std::vector<Matrix> jacobians(constraint->size());
    const Vector value = constraint->unwhitenedExpr(initial, &jacobians);
    const Vector sigmas = constraint->noiseModel()->sigmas();
    for (size_t row = 0; row < constraint->dim(); ++row) {
      std::vector<std::pair<Key, Matrix>> terms;
      double target = -value(row);
      for (size_t i = 0; i < constraint->size(); ++i) {
        const Key key = constraint->keys()[i];
        const Matrix block = jacobians[i].row(row);
        terms.emplace_back(key, block);
        target += (block * initial.at<Vector>(key))(0);
      }
      scalarConstraints.push_back(
          LinearConstraint::LessEqual(JacobianFactor(terms, Vector1(target)),
                                      Vector1(sigmas(row)))
              .createInequalityFactor());
    }
  }
  const ConstrainedOptProblem scalarProblem(
      problem.costs(), problem.eConstraints(), scalarConstraints);
  const auto state = AugmentedLagrangianOptimizer(scalarProblem, initial, al)
                         .optimizeWithState();
  if (!state.converged) {
    std::ostringstream message;
    message << "CMC QP AL fallback did not converge: stationarity="
            << state.augmentedLagrangianStationarity
            << ", violation=" << state.generalizedConstraintViolation;
    throw std::runtime_error(message.str());
  }
  ActiveSetSolverState result;
  result.values = state.values;
  result.iterations = state.iteration;
  result.converged = true;
  for (size_t i = 0; i < scalarConstraints.size(); ++i) {
    const auto& constraint = scalarConstraints[i];
    const Vector values = constraint->whitenedExpr(result.values);
    for (double value : values)
      result.activeInequalityRows.push_back(std::abs(value) < 1e-6 &&
                                            state.lambdaIneq[i] > 1e-8);
  }
  return result;
}
}  // namespace internal
}  // namespace gtdynamics
