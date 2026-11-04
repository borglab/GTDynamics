/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file WorkingFace.cpp
 * @brief Parameterize component-local working faces for unconstrained LM.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/WorkingFace.h>
#include <gtdynamics/utils/GraphUtils.h>

#include <Eigen/LU>
#include <algorithm>
#include <stdexcept>
namespace gtdynamics {
namespace internal {
using namespace gtsam;
WorkingFace::WorkingFace(Matrix rows) : rows_(std::move(rows)) {
  if (!rows_.allFinite() || !rows_.cols())
    throw std::invalid_argument("WorkingFace: invalid face matrix");
  for (Eigen::Index i = 0; i < rows_.rows(); ++i) {
    const double maximum = rows_.row(i).cwiseAbs().maxCoeff();
    if (maximum > 0) {
      rows_.row(i) /= maximum;
      rows_.row(i).normalize();
    }
  }
  if (!rows_.rows()) {
    basis_ = Matrix::Identity(rows_.cols(), rows_.cols());
  } else {
    const Eigen::FullPivLU<Matrix> factorization(rows_);
    const Eigen::Index freeDimension = rows_.cols() - factorization.rank();
    basis_ = freeDimension ? Matrix(factorization.kernel())
                           : Matrix(rows_.cols(), 0);
  }
  if (!basis_.allFinite())
    throw std::runtime_error("WorkingFace: nonfinite basis");
  sparseBasis_ = basis_.sparseView(0.0, 0.0);
}
Matrix WorkingFace::reduce(const Matrix& jacobian) const {
  return jacobian * sparseBasis_;
}
Vector WorkingFace::recover(const Vector& reduced) const {
  const Vector value = basis_ * reduced;
  const Vector residual = rows_ * value;
  if (!value.allFinite() || !residual.allFinite() ||
      (residual.size() && residual.lpNorm<Eigen::Infinity>() > 1e-7))
    throw std::runtime_error("WorkingFace: invalid recovered step");
  return value;
}
GaussianFactorGraph reduceWorkingFaces(const GaussianFactorGraph& cost,
                                       const WorkingFaces& faces) {
  const auto dimensions = cost.getKeyDimMap();
  for (const auto& [key, face] : faces)
    if (!dimensions.count(key) || dimensions.at(key) != face.dimension())
      throw std::invalid_argument("WorkingFace: face/cost dimension mismatch");
  GaussianFactorGraph projected;
  projected.reserve(cost.size());
  for (const auto& factor : cost) {
    if (!factor) continue;
    std::shared_ptr<JacobianFactor> converted;
    const auto* jacobian = dynamic_cast<const JacobianFactor*>(factor.get());
    if (!jacobian) {
      converted = std::make_shared<JacobianFactor>(*factor);
      jacobian = converted.get();
    }
    if (jacobian->isConstrained() || !jacobian->getb().allFinite())
      throw std::invalid_argument(
          "WorkingFace: objective must contain finite soft costs");
    bool affected = false;
    for (Key key : *jacobian) affected |= faces.count(key) != 0;
    if (!affected) {
      projected.push_back(factor);
      continue;
    }
    std::vector<std::pair<Key, Matrix>> terms;
    terms.reserve(jacobian->size());
    for (auto it = jacobian->begin(); it != jacobian->end(); ++it) {
      const auto face = faces.find(*it);
      if (face == faces.end())
        terms.emplace_back(*it, jacobian->getA(it));
      else if (face->second.freeDimension())
        terms.emplace_back(*it, face->second.reduce(jacobian->getA(it)));
    }
    if (!terms.empty())
      projected.emplace_shared<JacobianFactor>(terms, jacobian->getb(),
                                               jacobian->get_model());
  }
  return projected;
}
VectorValues recoverWorkingFaces(const VectorValues& reduced,
                                 const std::map<Key, size_t>& dimensions,
                                 const WorkingFaces& faces) {
  VectorValues result;
  for (const auto& [key, dimension] : dimensions) {
    const auto face = faces.find(key);
    const Vector value = face == faces.end()
                             ? reduced.at(key)
                             : face->second.recover(face->second.freeDimension()
                                                        ? reduced.at(key)
                                                        : Vector(0));
    if (!value.allFinite())
      throw std::runtime_error("WorkingFace: nonfinite step");
    result.insert(key, value);
  }
  return result;
}
VectorValues solveWorkingFaces(const GaussianFactorGraph& cost,
                               const WorkingFaces& faces,
                               const NonlinearOptimizerParams& params) {
  const auto projected = reduceWorkingFaces(cost, faces);
  NonlinearOptimizerParams reducedParams = params;
  const KeySet remainingKeys = projected.keys();
  if (params.ordering) {
    Ordering ordering;
    for (Key key : *params.ordering)
      if (remainingKeys.exists(key)) ordering.push_back(key);
    reducedParams.ordering = std::move(ordering);
  }
  VectorValues reduced;
  if (!remainingKeys.empty()) reduced = SolveLinear(projected, reducedParams);
  return recoverWorkingFaces(reduced, cost.getKeyDimMap(), faces);
}
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

}  // namespace internal
}  // namespace gtdynamics
