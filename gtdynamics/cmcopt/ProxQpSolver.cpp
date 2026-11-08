/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file ProxQpSolver.cpp
 * @brief Pack reduced QP blocks and validate sparse ProxQP solutions.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/ProxQpSolver.h>

#ifdef GTDYNAMICS_WITH_PROXQP
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <set>

namespace gtdynamics::internal {
namespace {
using namespace gtsam;
using Sparse = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using Triplet = Eigen::Triplet<double, int>;

// Retain exact nonzeros, including coefficients below sparseJacobian's cutoff.
void append(std::vector<Triplet>& entries, int row, int column, double value) {
  if (!std::isfinite(value))
    throw std::runtime_error("ProxQP: nonfinite QP coefficient");
  if (value != 0.0) entries.emplace_back(row, column, value);
}
}  // namespace

ProxQpResult solveProxQp(const QpProblem& problem, const Values& prototype,
                         const std::optional<Ordering>& requestedOrdering,
                         size_t maxIterations) {
  if (!problem.eConstraints().empty())
    throw std::invalid_argument(
        "ProxQP: CMC equalities must be eliminated by B");
  const Ordering ordering =
      requestedOrdering.value_or(Ordering(prototype.keys()));
  if (ordering.size() != prototype.size() ||
      std::set<Key>(ordering.begin(), ordering.end()).size() !=
          prototype.size())
    throw std::invalid_argument("ProxQP: ordering must contain every key once");
  std::map<Key, int> offsets;
  int dimension = 0;
  for (Key key : ordering) {
    if (!prototype.exists(key))
      throw std::invalid_argument("ProxQP: unknown ordering key");
    const auto& block = prototype.at<Vector>(key);
    if (block.size() > std::numeric_limits<int>::max() - dimension)
      throw std::invalid_argument("ProxQP: too many scalar variables");
    offsets.emplace(key, dimension);
    dimension += block.size();
  }

  ProxQpResult result;
  if (maxIterations == 0) {
    result.message = "ProxQP: iteration limit reached";
    return result;
  }
  // QpCost already owns local Hessian blocks. Scatter their upper triangles;
  // no dense global Jacobian or Hessian is ever formed.
  Vector linear = Vector::Zero(dimension);
  std::vector<Triplet> objectiveEntries;
  for (const auto& factor : problem.costs()) {
    const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
    if (!cost) throw std::invalid_argument("ProxQP: expected QpCost factors");
    const auto& hessian = cost->hessianFactor();
    size_t i = 0;
    for (auto rowKey = hessian.begin(); rowKey != hessian.end();
         ++rowKey, ++i) {
      const int rowOffset = offsets.at(*rowKey);
      const int rows = hessian.getDim(rowKey);
      linear.segment(rowOffset, rows) -= hessian.linearTerm(rowKey);
      size_t j = i;
      for (auto colKey = rowKey; colKey != hessian.end(); ++colKey, ++j) {
        const auto block = hessian.info().aboveDiagonalBlock(i, j);
        const int colOffset = offsets.at(*colKey);
        for (int col = 0; col < block.cols(); ++col)
          for (int row = 0; row < block.rows(); ++row) {
            if (i == j && row > col) continue;
            const int globalRow = rowOffset + row, globalCol = colOffset + col;
            append(objectiveEntries, std::min(globalRow, globalCol),
                   std::max(globalRow, globalCol), block(row, col));
          }
      }
    }
  }
  Sparse hessian(dimension, dimension);
  hessian.setFromTriplets(objectiveEntries.begin(), objectiveEntries.end());
  objectiveEntries.clear();
  objectiveEntries.shrink_to_fit();

  std::vector<Triplet> constraintEntries;
  std::vector<double> bounds;
  for (const auto& factor : problem.iConstraints()) {
    const auto constraint =
        std::dynamic_pointer_cast<LinearInequalityConstraintFactor>(factor);
    if (!constraint)
      throw std::invalid_argument("ProxQP: expected linear inequalities");
    const auto& source = constraint->linearConstraint();
    const auto& jacobian = source.factor();
    const double sign =
        source.sense() == LinearConstraint::Sense::GreaterEqual ? -1.0 : 1.0;
    const int rowOffset = bounds.size();
    for (auto key = jacobian.begin(); key != jacobian.end(); ++key) {
      const auto block = jacobian.getA(key);
      for (int col = 0; col < block.cols(); ++col)
        for (int row = 0; row < block.rows(); ++row)
          append(constraintEntries, rowOffset + row, offsets.at(*key) + col,
                 sign * block(row, col));
    }
    for (double value : jacobian.getb()) bounds.push_back(sign * value);
  }
  Sparse constraints(bounds.size(), dimension), equalities(0, dimension);
  constraints.setFromTriplets(constraintEntries.begin(),
                              constraintEntries.end());
  const Vector upper = Eigen::Map<const Vector>(bounds.data(), bounds.size());
  const Vector lower =
      Vector::Constant(bounds.size(), -std::numeric_limits<double>::infinity());
  if (!linear.allFinite() || !upper.allFinite())
    throw std::runtime_error("ProxQP: nonfinite QP coefficients");

  proxsuite::proxqp::sparse::QP<double, int> solver(dimension, 0,
                                                    bounds.size());
  solver.settings.max_iter = maxIterations;
  solver.settings.eps_abs = 1e-9;
  solver.settings.eps_rel = 0;
  solver.settings.check_duality_gap = true;
  solver.settings.eps_duality_gap_abs = 1e-8;
  solver.settings.eps_duality_gap_rel = 0;
  solver.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  solver.init(hessian, linear, equalities, Vector(0), constraints, lower,
              upper);
  solver.solve();
  result.iterations = solver.results.info.iter;
  if (solver.results.info.status !=
      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    result.message =
        "ProxQP: solve failed, status " +
        std::to_string(static_cast<int>(solver.results.info.status));
    return result;
  }
  const Vector& step = solver.results.x;
  const Vector& multipliers = solver.results.z;
  if (!step.allFinite() || !multipliers.allFinite()) {
    result.message = "ProxQP: nonfinite solution";
    return result;
  }
  VectorValues delta;
  for (Key key : ordering) {
    const Vector block =
        step.segment(offsets.at(key), prototype.at<Vector>(key).size());
    delta.insert(key, block);
    result.values.insert(key, block);
  }
  // Certify with original Gaussian factors, not just the packed normal matrix.
  Vector gradient = Vector::Zero(dimension);
  for (const auto& factor : problem.costs()) {
    const auto cost = std::static_pointer_cast<QpCost>(factor);
    for (Key key : cost->keys())
      gradient.segment(offsets.at(key), prototype.at<Vector>(key).size()) +=
          cost->gaussianFactor().gradient(key, delta);
  }
  const Vector residual = constraints * step - upper;
  const Vector stationarity = gradient + constraints.transpose() * multipliers;
  constexpr double tolerance = 1e-7;
  bool valid = stationarity.allFinite() &&
               stationarity.lpNorm<Eigen::Infinity>() <= tolerance &&
               std::isfinite(problem.costs().error(result.values));
  result.activeRows.resize(bounds.size());
  for (size_t row = 0; row < bounds.size(); ++row) {
    result.activeRows[row] =
        multipliers(row) > 0 && std::abs(residual(row)) <= tolerance;
    valid &= residual(row) <= tolerance && multipliers(row) >= -tolerance &&
             std::abs(residual(row) * multipliers(row)) <= tolerance;
    if (result.activeRows[row]) valid &= std::abs(residual(row)) <= tolerance;
  }
  result.converged = valid;
  if (!valid) result.message = "ProxQP: original-factor KKT checks failed";
  return result;
}
}  // namespace gtdynamics::internal
#endif  // GTDYNAMICS_WITH_PROXQP
