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
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/MultifrontalSolver.h>

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
using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// The dual coordinate updates are Dykstra projection onto an intersection of
// halfspaces. They do not require independent constraint rows (e.g. at
// corners).
struct Cone {
  int offset = 0;
  Matrix A;
  Matrix metricNormals;  // A M^{-1}, for the weighted projection.
  Vector rowNormSquared;
  Vector multipliers;
  std::vector<size_t> originalRows;
};

bool project(Cone& cone, const Vector& trial, Vector& projected,
             size_t maxSweeps, double tolerance) {
  projected = trial - cone.metricNormals.transpose() * cone.multipliers;
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
      projected.noalias() -= change * cone.metricNormals.row(j).transpose();
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

PointSplitAdmmWorkspace::PointSplitAdmmWorkspace() = default;
PointSplitAdmmWorkspace::~PointSplitAdmmWorkspace() = default;

PointSplitAdmmResult PointSplitAdmmWorkspace::solve(
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

  if (params.useConstraintSplitting)
    return solveConstraints(problem, prototype, requestedOrdering,
                            maxIterations, params);

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
      throw std::invalid_argument(
          "CMC ADMM: tangent inequalities must be homogeneous");
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

  // Add sqrt(rho M) unary factors to the original Gaussian objective.
  // The factorization stays fixed while the ADMM right-hand side changes.
  const auto factorizationStart = Clock::now();
  GaussianFactorGraph system;
  for (const auto& factor : problem.costs()) {
    const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
    if (!cost) throw std::invalid_argument("CMC ADMM: expected QpCost");
    system.add(cost->gaussianFactor());
  }
  const VectorValues gradientAtZero = system.gradientAtZero();
  Vector metric =
      Vector::Constant(dimension, params.useDiagonalMetric ? 1e-12 : 1.);
  if (params.useDiagonalMetric) {
    for (const auto& [key, diagonal] : system.hessianDiagonal()) {
      if (!diagonal.allFinite() || (diagonal.array() < 0.).any())
        throw std::invalid_argument(
            "CMC ADMM: invalid Hessian diagonal metric");
      metric.segment(offsets.at(key), diagonal.size()) =
          diagonal.cwiseMax(1e-12);
    }
  }
  for (auto& cone : cones) {
    const Vector inverse =
        metric.segment(cone.offset, cone.A.cols()).cwiseInverse();
    cone.metricNormals = cone.A * inverse.asDiagonal();
    for (int row = 0; row < cone.A.rows(); ++row)
      cone.rowNormSquared(row) =
          cone.A.row(row).dot(cone.metricNormals.row(row));
  }
  VectorValues rhs;
  for (Key key : ordering) {
    const auto dim = prototype.at<Vector>(key).size();
    if (!dim) continue;
    rhs.insert(key, Vector(Vector::Zero(dim)));
    system.emplace_shared<JacobianFactor>(
        key,
        Matrix((params.rho * metric.segment(offsets.at(key), dim))
                   .cwiseSqrt()
                   .asDiagonal()),
        Vector::Zero(dim));
  }
  for (const auto& [key, gradient] : gradientAtZero) {
    if (!gradient.allFinite())
      throw std::invalid_argument("CMC ADMM: nonfinite linear term");
    if (gradient.size() != prototype.at<Vector>(key).size())
      throw std::invalid_argument(
          "CMC ADMM: inconsistent cost block dimension");
  }
  GaussianBayesTree factorization;
  try {
    if (!solver_) {
      Ordering eliminationOrdering;
      if (requestedOrdering) {
        for (Key key : ordering)
          if (rhs.exists(key)) eliminationOrdering.push_back(key);
      } else {
        eliminationOrdering = Ordering::Colamd(system);
      }
      solver_ =
          std::make_unique<MultifrontalSolver>(system, eliminationOrdering);
    }
    // LM damping changes the numerical diagonal, but the symbolic tree and
    // clique load plans remain valid throughout this linearization.
    solver_->eliminateInPlace(system);
    factorization = solver_->computeBayesTree();
  } catch (const std::runtime_error& error) {
    solver_.reset();
    result.factorizationSeconds = seconds(factorizationStart);
    result.message =
        std::string("CMC ADMM: multifrontal factorization failed: ") +
        error.what();
    return result;
  }
  result.factorizationSeconds = seconds(factorizationStart);

  Vector x = Vector::Zero(dimension);
  Vector z = x, u = x;
  const double localTol = std::min(1e-10, params.absoluteTolerance);
  constexpr double kktTolerance = 1e-7;
  for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
    result.iterations = iteration + 1;
    for (auto& [key, block] : rhs) {
      const auto offset = offsets.at(key);
      block = params.rho * metric.segment(offset, block.size())
                               .cwiseProduct(z.segment(offset, block.size()) -
                                             u.segment(offset, block.size()));
      if (gradientAtZero.exists(key)) block -= gradientAtZero.at(key);
    }
    try {
      // H + rho M = R^T R. Both triangular solves ignore the stored RHS.
      const auto solution = factorization.backSubstitute(
          factorization.backSubstituteTranspose(rhs));
      for (const auto& [key, block] : solution)
        x.segment(offsets.at(key), block.size()) = block;
    } catch (const IndeterminateSystemException& error) {
      result.message =
          std::string("CMC ADMM: global solve failed: ") + error.what();
      return result;
    }
    const Vector oldZ = z;
    z = x + u;  // Unconstrained keys project onto the whole space.
    std::vector<int> projected(cones.size(), 0);
    const auto projectionStart = Clock::now();
    forIndependentComponents(
        cones.size(), params.projectionThreads, [&](size_t i) {
          auto& cone = cones[i];
          const int d = cone.A.cols();
          Vector answer;
          projected[i] = project(cone, z.segment(cone.offset, d), answer,
                                 params.maxProjectionSweeps, localTol)
                             ? 1
                             : 0;
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
    result.dualResidual =
        params.rho * metric.cwiseProduct(z - oldZ).lpNorm<Eigen::Infinity>();
    const double primalTol =
        params.absoluteTolerance +
        params.relativeTolerance * std::max({1., x.lpNorm<Eigen::Infinity>(),
                                             z.lpNorm<Eigen::Infinity>()});
    const double dualTol =
        params.absoluteTolerance +
        params.relativeTolerance * params.rho *
            std::max(1., metric.cwiseProduct(u).lpNorm<Eigen::Infinity>());
    if (result.primalResidual > primalTol || result.dualResidual > dualTol)
      continue;

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
        gradient.segment(offsets.at(key), prototype.at<Vector>(key).size()) +=
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
        valid &= slack(j) <= kktTolerance && dual(j) >= -kktTolerance &&
                 std::abs(slack(j) * dual(j)) <= kktTolerance;
        result.activeRows[cone.originalRows[j]] =
            dual(j) > 0. && std::abs(slack(j)) <= kktTolerance;
      }
    }
    result.stationarityResidual = gradient.lpNorm<Eigen::Infinity>();
    Values checkedValues;
    for (Key key : ordering) checkedValues.insert(key, delta.at(key));
    valid &= gradient.allFinite() &&
             result.stationarityResidual <= kktTolerance &&
             std::isfinite(problem.costs().error(checkedValues));
    if (!valid) continue;
    for (Key key : ordering) result.values.insert(key, delta.at(key));
    result.converged = true;
    return result;
  }
  result.message =
      "CMC ADMM: no KKT-certified solution after " +
      std::to_string(result.iterations) +
      " iterations (primal=" + std::to_string(result.primalResidual) +
      ", dual=" + std::to_string(result.dualResidual) + ", stationarity=" +
      (std::isnan(result.stationarityResidual)
           ? std::string("not evaluated")
           : std::to_string(result.stationarityResidual)) +
      ")";
  return result;
}

PointSplitAdmmResult solvePointSplitAdmm(
    const QpProblem& problem, const Values& prototype,
    const std::optional<Ordering>& requestedOrdering, size_t maxIterations,
    const PointSplitAdmmParams& params) {
  PointSplitAdmmWorkspace workspace;
  return workspace.solve(problem, prototype, requestedOrdering, maxIterations,
                         params);
}

}  // namespace gtdynamics::internal
