/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file ConstraintSplitAdmmSolver.cpp
 * @brief Scaled constraint-output ADMM with cached GTSAM multifrontal solves.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/AdmmSolver.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/MultifrontalSolver.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gtdynamics::internal {
namespace {
using namespace gtsam;
using Clock = std::chrono::steady_clock;

struct ConstraintBlock {
  Key key;
  size_t offset;
  Matrix original;
  Matrix scaled;
  Vector rowScale;
};

// Change coordinates in each original factor. Preserve square-root factors
// and their noise models; Hessian inputs retain their local Hessian form.
GaussianFactorGraph scaleCosts(const GaussianFactorGraph& original,
                               const VectorValues& scale) {
  GaussianFactorGraph result;
  for (const auto& factor : original) {
    if (const auto jacobian =
            std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      std::vector<std::pair<Key, Matrix>> terms;
      for (auto key = jacobian->begin(); key != jacobian->end(); ++key)
        terms.emplace_back(*key,
                           jacobian->getA(key) * scale.at(*key).asDiagonal());
      result.emplace_shared<JacobianFactor>(terms, jacobian->getb(),
                                            jacobian->get_model());
    } else if (const auto hessian =
                   std::dynamic_pointer_cast<HessianFactor>(factor)) {
      std::vector<Matrix> quadratic;
      std::vector<Vector> linear;
      size_t i = 0;
      for (auto row = hessian->begin(); row != hessian->end(); ++row, ++i) {
        linear.emplace_back(scale.at(*row).asDiagonal() *
                            hessian->linearTerm(row));
        size_t j = i;
        for (auto column = row; column != hessian->end(); ++column, ++j)
          quadratic.emplace_back(scale.at(*row).asDiagonal() *
                                 hessian->info().aboveDiagonalBlock(i, j) *
                                 scale.at(*column).asDiagonal());
      }
      result.emplace_shared<HessianFactor>(hessian->keys(), quadratic, linear,
                                           hessian->constantTerm());
    } else {
      throw std::invalid_argument(
          "CMC constraint ADMM: unsupported Gaussian cost");
    }
  }
  return result;
}
}  // namespace

AdmmResult AdmmWorkspace::solveConstraints(
    const QpProblem& problem, const Values& prototype,
    const std::optional<Ordering>& requestedOrdering, size_t maxIterations,
    const AdmmParams& params) {
  // Common formulation, parameter and ordering validation precedes this call.
  AdmmResult result;
  const auto start = Clock::now();
  GaussianFactorGraph original;
  for (const auto& factor : problem.costs()) {
    const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
    if (!cost)
      throw std::invalid_argument("CMC constraint ADMM: expected QpCost");
    original.add(cost->gaussianFactor());
  }
  const auto diagonal = original.hessianDiagonal();
  VectorValues scale, zero;
  for (Key key : prototype.keys()) {
    const int dimension = prototype.at<Vector>(key).size();
    if (!dimension) continue;
    Vector values = Vector::Ones(dimension);
    if (params.useDiagonalMetric) {
      const Vector entry = diagonal.exists(key)
                               ? diagonal.at(key)
                               : Vector(Vector::Zero(dimension));
      if (!entry.allFinite() || entry.size() != dimension ||
          (entry.array() < 0.).any())
        throw std::invalid_argument(
            "CMC constraint ADMM: invalid Hessian diagonal");
      values = entry.cwiseMax(1e-12).cwiseSqrt().cwiseInverse();
    }
    scale.insert(key, values);
    zero.insert(key, Vector::Zero(dimension));
  }
  GaussianFactorGraph system =
      params.useDiagonalMetric ? scaleCosts(original, scale) : original;
  const auto gradientAtZero = system.gradientAtZero();
  for (const auto& [key, gradient] : gradientAtZero)
    if (!scale.exists(key) || gradient.size() != scale.at(key).size() ||
        !gradient.allFinite())
      throw std::invalid_argument(
          "CMC constraint ADMM: invalid objective gradient");

  std::vector<ConstraintBlock> blocks;
  size_t rowCount = 0;
  for (const auto& factor : problem.iConstraints()) {
    const auto inequality =
        std::dynamic_pointer_cast<LinearInequalityConstraintFactor>(factor);
    if (!inequality)
      throw std::invalid_argument(
          "CMC constraint ADMM: expected linear inequality");
    const auto& constraint = inequality->linearConstraint();
    const auto& jacobian = constraint.factor();
    if (jacobian.size() != 1)
      throw std::invalid_argument(
          "CMC constraint ADMM: inequality must be point-local");
    const Key key = *jacobian.begin();
    if (!scale.exists(key))
      throw std::invalid_argument(
          "CMC constraint ADMM: unknown inequality key");
    if (!jacobian.getb().allFinite() ||
        (jacobian.getb().size() &&
         jacobian.getb().cwiseAbs().maxCoeff() > 1e-12))
      throw std::invalid_argument(
          "CMC constraint ADMM: expected homogeneous rows");
    const double sign =
        constraint.sense() == LinearConstraint::Sense::GreaterEqual ? -1. : 1.;
    ConstraintBlock block{key, rowCount, sign * jacobian.getA(), Matrix(),
                          Vector()};
    if (!block.original.allFinite() ||
        block.original.cols() != scale.at(key).size())
      throw std::invalid_argument("CMC constraint ADMM: malformed constraint");
    block.scaled = block.original * scale.at(key).asDiagonal();
    block.rowScale = Vector::Ones(block.scaled.rows());
    if (params.useDiagonalMetric) {
      for (int row = 0; row < block.scaled.rows(); ++row) {
        const double norm = block.scaled.row(row).norm();
        if (norm > 0.) block.rowScale(row) = 1. / std::max(norm, 1e-12);
      }
      block.scaled = block.rowScale.asDiagonal() * block.scaled;
    }
    if (!block.scaled.allFinite())
      throw std::invalid_argument("CMC constraint ADMM: nonfinite scaled rows");
    rowCount += block.scaled.rows();
    system.emplace_shared<JacobianFactor>(key,
                                          std::sqrt(params.rho) * block.scaled,
                                          Vector::Zero(block.scaled.rows()));
    blocks.push_back(std::move(block));
  }

  GaussianBayesTree tree;
  try {
    if (!solver_) {
      Ordering ordering;
      if (requestedOrdering) {
        for (Key key : *requestedOrdering)
          if (zero.exists(key)) ordering.push_back(key);
      } else {
        ordering = Ordering::Colamd(system);
      }
      solver_ = std::make_unique<MultifrontalSolver>(system, ordering);
    }
    solver_->eliminateInPlace(system);
    tree = solver_->computeBayesTree();
  } catch (const std::runtime_error& error) {
    solver_.reset();
    result.message =
        std::string("CMC constraint ADMM: factorization failed: ") +
        error.what();
    result.factorizationSeconds =
        std::chrono::duration<double>(Clock::now() - start).count();
    return result;
  }
  result.factorizationSeconds =
      std::chrono::duration<double>(Clock::now() - start).count();
  Vector z = Vector::Zero(rowCount), u = z, constraintValues = z;
  VectorValues rhs = zero, scaledSolution = zero;
  double violation = 0., complementarity = 0.;
  constexpr double kktTolerance = 1e-7;
  for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
    result.iterations = iteration + 1;
    for (auto& [key, value] : rhs) {
      value.setZero();
      if (gradientAtZero.exists(key)) value -= gradientAtZero.at(key);
    }
    for (const auto& block : blocks) {
      const int rows = block.scaled.rows();
      rhs.at(block.key).noalias() +=
          params.rho * block.scaled.transpose() *
          (z.segment(block.offset, rows) - u.segment(block.offset, rows));
    }
    try {
      scaledSolution = tree.backSubstitute(tree.backSubstituteTranspose(rhs));
    } catch (const std::runtime_error& error) {
      result.message =
          std::string("CMC constraint ADMM: global solve failed: ") +
          error.what();
      return result;
    }
    for (const auto& [key, value] : scaledSolution)
      if (!value.allFinite()) {
        result.message = "CMC constraint ADMM: nonfinite global solution";
        return result;
      }
    for (const auto& block : blocks)
      constraintValues.segment(block.offset, block.scaled.rows()) =
          block.scaled * scaledSolution.at(block.key);
    const auto projectionStart = Clock::now();
    z = (constraintValues + u).cwiseMin(0.);
    u += constraintValues - z;
    result.projectionSeconds +=
        std::chrono::duration<double>(Clock::now() - projectionStart).count();
    if (!z.allFinite() || !u.allFinite()) {
      result.message = "CMC constraint ADMM: nonfinite split state";
      return result;
    }
    result.primalResidual =
        rowCount ? (constraintValues - z).lpNorm<Eigen::Infinity>() : 0.;
    if (iteration % 25 != 0 && iteration + 1 != maxIterations && rowCount)
      continue;

    // Certify in original physical units. Row-scaled duals transform as
    // lambda = rho E u; variable scaling cancels in the physical gradient.
    VectorValues physical = zero;
    for (auto& [key, value] : physical)
      value = scale.at(key).cwiseProduct(scaledSolution.at(key));
    auto gradient = original.gradient(physical);
    result.activeRows.assign(rowCount, false);
    bool valid = true;
    violation = complementarity = 0.;
    for (const auto& block : blocks) {
      const Vector slack = block.original * physical.at(block.key);
      const Vector multipliers =
          params.rho * block.rowScale.cwiseProduct(
                           u.segment(block.offset, block.scaled.rows()));
      if (!gradient.exists(block.key))
        gradient.insert(block.key, Vector::Zero(scale.at(block.key).size()));
      gradient.at(block.key).noalias() +=
          block.original.transpose() * multipliers;
      valid &= slack.allFinite() && multipliers.allFinite();
      for (int row = 0; row < slack.size(); ++row) {
        violation = std::max(violation, slack(row));
        complementarity =
            std::max(complementarity, std::abs(slack(row) * multipliers(row)));
        valid &= multipliers(row) >= -kktTolerance;
        result.activeRows[block.offset + row] =
            multipliers(row) > 0. && std::abs(slack(row)) <= kktTolerance;
      }
    }
    result.stationarityResidual = 0.;
    for (const auto& [key, value] : gradient) {
      valid &= value.allFinite();
      result.stationarityResidual = std::max(result.stationarityResidual,
                                             value.lpNorm<Eigen::Infinity>());
    }
    result.dualResidual = result.stationarityResidual;
    valid &= violation <= kktTolerance && complementarity <= kktTolerance &&
             result.stationarityResidual <= kktTolerance &&
             std::isfinite(original.error(physical));
    if (valid) {
      for (Key key : prototype.keys())
        result.values.insert(
            key, physical.exists(key) ? physical.at(key) : Vector(0));
      result.converged = true;
      return result;
    }
  }
  std::ostringstream message;
  message << "CMC constraint ADMM: no original-factor KKT certificate after "
          << result.iterations << " iterations (violation=" << violation
          << ", stationarity=" << result.stationarityResidual
          << ", complementarity=" << complementarity << ")";
  result.message = message.str();
  return result;
}
}  // namespace gtdynamics::internal
