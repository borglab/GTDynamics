/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CMTangentSpace.cpp
 * @brief Share component geometry between cone projection and LM face checks.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/CMTangentSpace.h>

#include <stdexcept>

namespace gtdynamics {
CMTangentSpace::CMTangentSpace(const LinearIConstraintMap& tangentConstraints,
                               const LinearIConstraintMap& ambientConstraints) {
  LinearInequalityConstraints coneConstraints;
  for (const auto& [index, constraint] : tangentConstraints) {
    const auto factor = constraint->createL2Factor();
    if (factor->size() != 1 || !factor->getb().isZero(0.0) ||
        factor->isConstrained())
      throw std::invalid_argument(
          "CMC tangent inequalities must be unary, homogeneous, and "
          "soft-scaled");

    // Raw rows serve directional checks; whitened rows serve QP assembly.
    const auto whitened = factor->whiten();
    Block block{factor->getA(factor->begin()), whitened.getA(whitened.begin()),
                factor->get_model(), nullptr};
    if (!block.raw.allFinite() || !block.whitened.allFinite())
      throw std::invalid_argument("CMC tangent inequality has nonfinite rows");
    const auto ambient = ambientConstraints.find(index);
    if (ambient != ambientConstraints.end())
      block.ambient =
          std::make_shared<JacobianFactor>(*ambient->second->createL2Factor());
    coneConstraints.emplace_shared<JacobianLinearInequalityConstraint>(
        std::make_shared<JacobianFactor>(
            1, block.raw, Vector::Zero(block.raw.rows()), block.noise));
    blocks_.emplace(index, std::move(block));
    indices_.insert(index);
  }

  cone_ = std::make_shared<TangentCone>(coneConstraints);
}

CMTangentSpace::CMTangentSpace(const TangentSpaceBasis::shared_ptr& basis,
                               const LinearIConstraintMap& tangent,
                               const LinearIConstraintMap& ambient,
                               const CMTangentSpace* predecessor)
    : CMTangentSpace(tangent, ambient) {
  basis_ = basis;
  if (predecessor) metricCache_ = predecessor->metricCache_;
}

TangentSpaceMetric::shared_ptr CMTangentSpace::projectionMetric(
    const Values& values, const VectorValues& precisions, bool whiten) const {
  std::lock_guard<std::mutex> lock(metricCache_->mutex);
  if (!metricCache_->metric ||
      !metricCache_->metric->matches(precisions, whiten))
    metricCache_->metric = std::make_shared<TangentSpaceMetric>(
        *basis_, values, precisions, whiten);
  return metricCache_->metric;
}

const Matrix& CMTangentSpace::rows(size_t index, bool whitened) const {
  const auto& block = blocks_.at(index);
  return whitened ? block.whitened : block.raw;
}

IndexSet CMTangentSpace::violated(const Vector& delta, double tolerance) const {
  IndexSet result;
  for (const auto& [index, block] : blocks_)
    if (block.raw.rows() && (block.raw * delta).maxCoeff() > tolerance)
      result.insert(index);
  return result;
}

bool CMTangentSpace::active(const Vector& delta, const IndexSet& indices,
                            double tolerance) const {
  for (size_t index : indices)
    if ((rows(index, true) * delta).norm() > tolerance) return false;
  return true;
}

std::pair<IndexSet, Vector> CMTangentSpace::project(const Vector& delta) const {
  auto result = cone_->project(delta);

  // Convert cone-local indices back to the original nonlinear block indices.
  const std::vector<size_t> original(indices_.begin(), indices_.end());
  IndexSet indices;
  for (size_t index : result.first) indices.insert(original.at(index));
  return {indices, std::move(result.second)};
}

IndexSet CMTangentSpace::blockingIndices(const VectorValues& direction) const {
  IndexSet indices;
  for (const auto& [index, block] : blocks_) {
    if (!block.ambient)
      throw std::logic_error("Ambient constraint linearization is unavailable");
    Vector derivative = Vector::Zero(block.raw.rows());
    for (auto it = block.ambient->begin(); it != block.ambient->end(); ++it)
      if (direction.exists(*it))
        derivative += block.ambient->getA(it) * direction.at(*it);
    if ((derivative.array() > 1e-5).any()) indices.insert(index);
  }

  return indices;
}

}  // namespace gtdynamics
