/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file PointSplitAdmmSolver.cpp
 * @brief Factor-once global step and parallel per-point cone projections.
 */
#include <gtdynamics/cmcopt/PointSplitAdmmSolver.h>
#include <gtdynamics/constrained/internal/IndependentComponents.h>

#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>

#include <Eigen/SparseCholesky>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtdynamics::internal {
namespace {
using namespace gtsam;
using Sparse = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using Triplet = Eigen::Triplet<double, int>;
using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

void appendSymmetric(std::vector<Triplet>& entries, int row, int col,
                     double value) {
  if (!std::isfinite(value))
    throw std::invalid_argument("CMC ADMM: nonfinite Hessian coefficient");
  if (value == 0.) return;
  entries.emplace_back(row, col, value);
  if (row != col) entries.emplace_back(col, row, value);
}

// The dual coordinate updates are Dykstra projection onto an intersection of
// halfspaces. They do not require independent constraint rows (e.g. at corners).
struct Cone {
  int offset = 0;
  Matrix A;
  Vector rowNormSquared;
  Vector multipliers;
  std::vector<size_t> originalRows;
};

bool project(Cone& cone, const Vector& trial, Vector& projected,
             size_t maxSweeps, double tolerance) {
  projected = trial - cone.A.transpose() * cone.multipliers;
  for (size_t sweep = 0; sweep < maxSweeps; ++sweep) {
    double maxChange = 0.;
    for (int j = 0; j < cone.A.rows(); ++j) {
      if (cone.rowNormSquared(j) == 0.) continue;
      const double old = cone.multipliers(j);
      const double next = std::max(
          0., old + cone.A.row(j).dot(projected) / cone.rowNormSquared(j));
      cone.multipliers(j) = next;
      const double change = next - old;
      maxChange = std::max(maxChange, std::abs(change));
      projected.noalias() -= change * cone.A.row(j).transpose();
    }
    if (maxChange > tolerance) continue;
    // Coordinate changes alone do not establish primal/dual feasibility.
    bool valid = true;
    for (int j = 0; j < cone.A.rows(); ++j) {
      const double slack = cone.A.row(j).dot(projected);
      valid &= slack <= tolerance &&
               std::abs(cone.multipliers(j) * slack) <= tolerance;
    }
    if (valid && projected.allFinite()) return true;
  }
  return false;
}

struct Row {
  size_t original;
  Vector normal;
};
}  // namespace

PointSplitAdmmResult solvePointSplitAdmm(
    const QpProblem& problem, const Values& prototype,
    const std::optional<Ordering>& requestedOrdering, size_t maxIterations,
    const PointSplitAdmmParams& params) {
  if (!problem.eConstraints().empty())
    throw std::invalid_argument("CMC ADMM: equalities must be eliminated by B");
  if (!std::isfinite(params.rho) || params.rho <= 0. ||
      !std::isfinite(params.absoluteTolerance) ||
      params.absoluteTolerance <= 0. ||
      !std::isfinite(params.relativeTolerance) ||
      params.relativeTolerance < 0. || !params.maxProjectionSweeps ||
      !params.projectionThreads)
    throw std::invalid_argument("CMC ADMM: invalid solver parameters");

  const Ordering ordering =
      requestedOrdering.value_or(Ordering(prototype.keys()));
  if (ordering.size() != prototype.size() ||
      std::set<Key>(ordering.begin(), ordering.end()).size() !=
          prototype.size())
    throw std::invalid_argument("CMC ADMM: incomplete or duplicate ordering");

  std::map<Key, int> offsets;
  int dimension = 0;
  for (Key key : ordering) {
    if (!prototype.exists(key))
      throw std::invalid_argument("CMC ADMM: unknown ordering key");
    const auto& block = prototype.at<Vector>(key);
    if (block.size() > std::numeric_limits<int>::max() - dimension)
      throw std::invalid_argument("CMC ADMM: scalar dimension overflow");
    offsets.emplace(key, dimension);
    dimension += block.size();
  }

  PointSplitAdmmResult result;
  if (!maxIterations) {
    result.message = "CMC ADMM: outer iteration limit reached";
    return result;
  }
  if (!dimension) {
    result.converged = true;
    return result;
  }

  // Preserve the local Hessian blocks supplied by QpCost. We deliberately
  // measure a sparse normal-equation backend first; a cached GTSAM square-root
  // multifrontal solve can later be substituted without changing the split.
  std::vector<Triplet> entries;
  Vector linear = Vector::Zero(dimension);
  for (const auto& factor : problem.costs()) {
    const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
    if (!cost) throw std::invalid_argument("CMC ADMM: expected QpCost");
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
            appendSymmetric(entries, rowOffset + row, colOffset + col,
                            block(row, col));
          }
      }
    }
  }
  for (int i = 0; i < dimension; ++i)
    entries.emplace_back(i, i, params.rho);
  if (!linear.allFinite())
    throw std::invalid_argument("CMC ADMM: nonfinite linear term");

  // CMC constructs one-key tangent-cone rows. Group them by reduced key,
  // preserving the exact original row indices for nonlinear retraction.
  std::map<Key, std::vector<Row>> grouped;
  size_t rowCount = 0;
  for (const auto& factor : problem.iConstraints()) {
    const auto inequality =
        std::dynamic_pointer_cast<LinearInequalityConstraintFactor>(factor);
    if (!inequality)
      throw std::invalid_argument("CMC ADMM: expected linear inequality");
    const auto& source = inequality->linearConstraint();
    const auto& jacobian = source.factor();
    if (jacobian.size() != 1)
      throw std::invalid_argument("CMC ADMM: inequality must be point-local");
    const Key key = *jacobian.begin();
    if (!offsets.count(key))
      throw std::invalid_argument("CMC ADMM: unknown inequality key");
    const double sign =
        source.sense() == LinearConstraint::Sense::GreaterEqual ? -1. : 1.;
    const Vector rhs = sign * jacobian.getb();
    if (!rhs.allFinite() || rhs.cwiseAbs().maxCoeff() > 1e-12)
      throw std::invalid_argument("CMC ADMM: tangent inequalities must be homogeneous");
    const Matrix A = sign * jacobian.getA(jacobian.begin());
    if (!A.allFinite() || A.cols() != prototype.at<Vector>(key).size())
      throw std::invalid_argument("CMC ADMM: malformed cone rows");
    auto& rows = grouped[key];
    for (int j = 0; j < A.rows(); ++j)
      rows.push_back({rowCount++, Vector(A.row(j).transpose())});
  }
  std::vector<Cone> cones;
  cones.reserve(grouped.size());
  for (auto& [key, rows] : grouped) {
    const int d = prototype.at<Vector>(key).size();
    Cone cone;
    cone.offset = offsets.at(key);
    cone.A.resize(rows.size(), d);
    cone.rowNormSquared.resize(rows.size());
    cone.multipliers = Vector::Zero(rows.size());
    for (size_t j = 0; j < rows.size(); ++j) {
      cone.A.row(j) = rows[j].normal.transpose();
      cone.rowNormSquared(j) = rows[j].normal.squaredNorm();
      cone.originalRows.push_back(rows[j].original);
    }
    cones.push_back(std::move(cone));
  }

  const auto factorizationStart = Clock::now();
  Sparse system(dimension, dimension);
  system.setFromTriplets(entries.begin(), entries.end());
  Eigen::SimplicialLDLT<Sparse> solver;
  solver.compute(system);
  result.factorizationSeconds = seconds(factorizationStart);
  if (solver.info() != Eigen::Success) {
    result.message = "CMC ADMM: sparse factorization failed";
    return result;
  }

  Vector x = Vector::Zero(dimension);
  Vector z = x, u = x;
  const double localTol = std::min(1e-10, params.absoluteTolerance);
  constexpr double kktTolerance = 1e-7;
  for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
    x = solver.solve(params.rho * (z - u) - linear);
    result.iterations = iteration + 1;
    if (solver.info() != Eigen::Success || !x.allFinite()) {
      result.message = "CMC ADMM: global solve failed";
      return result;
    }
    const Vector oldZ = z;
    z = x + u;  // Unconstrained keys project onto the whole space.
    std::vector<int> projected(cones.size(), 0);
    const auto projectionStart = Clock::now();
    forIndependentComponents(cones.size(), params.projectionThreads,
                             [&](size_t i) {
      auto& cone = cones[i];
      const int d = cone.A.cols();
      Vector answer;
      projected[i] = project(cone, z.segment(cone.offset, d), answer,
                             params.maxProjectionSweeps, localTol) ? 1 : 0;
      if (projected[i]) z.segment(cone.offset, d) = answer;
    });
    result.projectionSeconds += seconds(projectionStart);
    if (std::any_of(projected.begin(), projected.end(),
                    [](int ok) { return ok == 0; })) {
      result.message = "CMC ADMM: local cone projection did not converge";
      return result;
    }
    u += x - z;
    result.primalResidual = (x - z).lpNorm<Eigen::Infinity>();
    result.dualResidual = params.rho *
                          (z - oldZ).lpNorm<Eigen::Infinity>();
    const double primalTol = params.absoluteTolerance +
        params.relativeTolerance *
            std::max({1., x.lpNorm<Eigen::Infinity>(),
                      z.lpNorm<Eigen::Infinity>()});
    const double dualTol = params.absoluteTolerance +
        params.relativeTolerance * params.rho *
            std::max(1., u.lpNorm<Eigen::Infinity>());
    if (result.primalResidual > primalTol ||
        result.dualResidual > dualTol) continue;

    // ADMM residuals are necessary, not sufficient: certify with the original
    // square-root factors, cone slack, dual feasibility and complementarity.
    VectorValues delta;
    for (Key key : ordering) {
      const int size = prototype.at<Vector>(key).size();
      delta.insert(key, Vector(z.segment(offsets.at(key), size)));
    }
    Vector gradient = Vector::Zero(dimension);
    for (const auto& factor : problem.costs()) {
      const auto cost = std::static_pointer_cast<QpCost>(factor);
      for (Key key : cost->keys())
        gradient.segment(offsets.at(key),
                         prototype.at<Vector>(key).size()) +=
            cost->gaussianFactor().gradient(key, delta);
    }
    result.activeRows.assign(rowCount, false);
    bool valid = gradient.allFinite();
    for (const auto& cone : cones) {
      const Vector local = z.segment(cone.offset, cone.A.cols());
      const Vector slack = cone.A * local;
      const Vector dual = params.rho * cone.multipliers;
      gradient.segment(cone.offset, cone.A.cols()).noalias() +=
          cone.A.transpose() * dual;
      for (int j = 0; j < slack.size(); ++j) {
        valid &= slack(j) <= kktTolerance &&
                 dual(j) >= -kktTolerance &&
                 std::abs(slack(j) * dual(j)) <= kktTolerance;
        result.activeRows[cone.originalRows[j]] =
            dual(j) > 0. && std::abs(slack(j)) <= kktTolerance;
      }
    }
    result.stationarityResidual = gradient.lpNorm<Eigen::Infinity>();
    valid &= gradient.allFinite() &&
             result.stationarityResidual <= kktTolerance &&
             std::isfinite(problem.costs().error(delta));
    if (!valid) continue;
    for (Key key : ordering)
      result.values.insert(key, delta.at(key));
    result.converged = true;
    return result;
  }
  result.message = "CMC ADMM: no KKT-certified solution after " +
                   std::to_string(result.iterations) + " iterations (primal=" +
                   std::to_string(result.primalResidual) + ", dual=" +
                   std::to_string(result.dualResidual) + ", stationarity=" +
                   std::to_string(result.stationarityResidual) + ")";
  return result;
}

}  // namespace gtdynamics::internal
