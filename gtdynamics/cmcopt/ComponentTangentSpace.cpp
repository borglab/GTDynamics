/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ComponentTangentSpace.cpp
 * @brief Substitute local active-face null spaces into Gaussian costs.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/ComponentTangentSpace.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/linearExceptions.h>

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <algorithm>
#include <map>
#include <stdexcept>
#include <vector>

namespace gtdynamics {
namespace internal {
namespace {
using namespace gtsam;
using Bases = std::map<Key, Matrix>;

// Use the same unit rows for rank detection and recovery checks. Otherwise a
// large arbitrary face scale can amplify roundoff into a false violation.
Bases normalizedFaces(Bases blocks) {
  for (auto& [key, rows] : blocks) {
    if (!rows.allFinite() || rows.cols() == 0)
      throw std::invalid_argument("ComponentTangentSpace: invalid face matrix");
    for (Eigen::Index i = 0; i < rows.rows(); ++i) {
      const double maximum = rows.row(i).cwiseAbs().maxCoeff();
      if (maximum > 0) {
        rows.row(i) /= maximum;
        rows.row(i).normalize();
      }
    }
  }
  return blocks;
}

// FullPivLU::kernel is also used by GTDynamics' dense tangent bases.
Bases localBases(const Bases& blocks) {
  Bases bases;
  for (const auto& [key, rows] : blocks) {
    if (rows.rows() == 0) continue;
    const Eigen::FullPivLU<Matrix> factorization(rows);
    // Eigen returns a single zero column for a trivial kernel. Represent that
    // case explicitly as zero DOF rather than an unconstrained dummy variable.
    const Eigen::Index freeDimension = rows.cols() - factorization.rank();
    Matrix basis =
        freeDimension ? Matrix(factorization.kernel()) : Matrix(rows.cols(), 0);
    if (!basis.allFinite())
      throw std::runtime_error("ComponentTangentSpace: nonfinite basis");
    bases.emplace(key, std::move(basis));
  }
  return bases;
}

// Substitute delta_key = N_key * reduced_key in the original damped costs.
// Unaffected factors are shared; no identity matrix products are needed.
GaussianFactorGraph projectedCosts(const GaussianFactorGraph& cost,
                                   const Bases& bases) {
  // FullPivLU kernels contain an identity block and only rank-many dense
  // rows. Retain every nonzero coefficient, including arbitrarily small ones,
  // while avoiding dense products with the structural zeros.
  std::map<Key, Eigen::SparseMatrix<double>> sparseBases;
  for (const auto& [key, basis] : bases)
    sparseBases.emplace(key, basis.sparseView(0.0, 0.0));
  GaussianFactorGraph projected;
  projected.reserve(cost.size());
  for (const auto& factor : cost) {
    if (!factor) continue;
    // Native GTSAM converts Hessian costs when a caller supplies one. Ordinary
    // nonlinear least-squares linearizations already supply Jacobian factors.
    std::shared_ptr<JacobianFactor> converted;
    const auto* jacobian = dynamic_cast<const JacobianFactor*>(factor.get());
    if (!jacobian) {
      converted = std::make_shared<JacobianFactor>(*factor);
      jacobian = converted.get();
    }
    if (jacobian->isConstrained() || !jacobian->getb().allFinite())
      throw std::invalid_argument(
          "ComponentTangentSpace: the objective must contain finite soft "
          "costs");
    bool affected = false;
    for (Key key : *jacobian) affected |= bases.count(key) != 0;
    if (!affected) {
      projected.push_back(factor);
      continue;
    }
    std::vector<std::pair<Key, Matrix>> terms;
    terms.reserve(jacobian->size());
    for (auto it = jacobian->begin(); it != jacobian->end(); ++it) {
      const auto basis = bases.find(*it);
      if (basis == bases.end()) {
        terms.emplace_back(*it, jacobian->getA(it));
      } else if (basis->second.cols()) {
        terms.emplace_back(*it, jacobian->getA(it) * sparseBases.at(*it));
      }
    }
    // A fully fixed factor is a constant offset and has no effect on the
    // minimizer. Leaving a keyless factor in the graph would make complete
    // elimination report a nonempty remainder.
    if (terms.empty()) continue;
    projected.emplace_shared<JacobianFactor>(terms, jacobian->getb(),
                                             jacobian->get_model());
  }
  return projected;
}
}  // namespace

gtsam::GaussianFactorGraph compactGaussianCosts(
    const gtsam::GaussianFactorGraph& cost) {
  using namespace gtsam;
  struct Bucket {
    size_t index = 0;
    GaussianFactorGraph factors;
  };
  std::map<KeyVector, Bucket> buckets;
  GaussianFactorGraph compact;
  compact.reserve(cost.size());
  for (const auto& factor : cost) {
    const auto* jacobian = dynamic_cast<const JacobianFactor*>(factor.get());
    if (!jacobian || jacobian->isConstrained()) {
      compact.push_back(factor);
      continue;
    }
    KeyVector scope = jacobian->keys();
    std::sort(scope.begin(), scope.end());
    auto [entry, inserted] = buckets.try_emplace(scope);
    if (inserted) {
      entry->second.index = compact.size();
      compact.push_back(GaussianFactor::shared_ptr{});
    }
    entry->second.factors.push_back(factor);
  }
  for (const auto& [scope, bucket] : buckets) {
    if (bucket.factors.size() == 1) {
      compact[bucket.index] = bucket.factors.front();
    } else {
      compact[bucket.index] = std::make_shared<JacobianFactor>(bucket.factors);
    }
  }
  return compact;
}

ComponentTangentSpace::ComponentTangentSpace(
    const std::map<gtsam::Key, gtsam::Matrix>& faces)
    : faces_(normalizedFaces(faces)), bases_(localBases(faces_)) {}

gtsam::GaussianFactorGraph ComponentTangentSpace::reduce(
    const gtsam::GaussianFactorGraph& cost) const {
  const auto dimensions = cost.getKeyDimMap();
  for (const auto& [key, basis] : bases_) {
    if (!dimensions.count(key) || dimensions.at(key) != size_t(basis.rows()))
      throw std::invalid_argument(
          "ComponentTangentSpace: face/cost dimension mismatch");
  }
  return projectedCosts(cost, bases_);
}

gtsam::VectorValues ComponentTangentSpace::recover(
    const gtsam::VectorValues& reduced,
    const std::map<gtsam::Key, size_t>& dimensions) const {
  using namespace gtsam;
  VectorValues result;
  for (const auto& [key, dimension] : dimensions) {
    const auto basis = bases_.find(key);
    Vector value;
    if (basis == bases_.end()) {
      value = reduced.at(key);
    } else if (basis->second.cols()) {
      value = basis->second * reduced.at(key);
    } else {
      value = Vector::Zero(dimension);
    }
    if (!value.allFinite())
      throw std::runtime_error(
          "ComponentTangentSpace: nonfinite recovered step");
    result.insert(key, value);
  }
  for (const auto& [key, rows] : faces_) {
    const Vector residual = rows * result.at(key);
    if (!residual.allFinite() ||
        (residual.size() && residual.lpNorm<Eigen::Infinity>() > 1e-7))
      throw std::runtime_error(
          "ComponentTangentSpace: recovered step violates a face");
  }
  return result;
}

gtsam::VectorValues ComponentTangentSpace::solve(
    const gtsam::GaussianFactorGraph& cost,
    const gtsam::NonlinearOptimizerParams& params) const {
  using namespace gtsam;
  const auto projected = reduce(cost);
  NonlinearOptimizerParams reducedParams = params;
  const KeySet remainingKeys = projected.keys();
  if (params.ordering) {
    Ordering ordering;
    for (Key key : *params.ordering)
      if (remainingKeys.exists(key)) ordering.push_back(key);
    reducedParams.ordering = std::move(ordering);
  }
  // This is the only trajectory-wide solve: every factor is an ordinary cost.
  // Damping was added before reduction, so its metric is transformed as D*N.
  VectorValues reduced;
  if (!remainingKeys.empty()) reduced = SolveLinear(projected, reducedParams);
  return recover(reduced, cost.getKeyDimMap());
}
}  // namespace internal
}  // namespace gtdynamics
