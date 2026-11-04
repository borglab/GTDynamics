/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IEManifoldLinearization.cpp
 * @brief Share component geometry between cone projection and LM face checks.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/IEManifoldLinearization.h>

#include <stdexcept>

namespace gtdynamics {
IEManifoldLinearization::IEManifoldLinearization(
    const LinearIConstraintMap& tangentConstraints,
    const LinearIConstraintMap& ambientConstraints) {
  LinearInequalityConstraints coneConstraints;
  for (const auto& [index, constraint] : tangentConstraints) {
    const auto factor = constraint->createL2Factor();
    if (factor->size() != 1 || !factor->getb().isZero(0.0) ||
        factor->isConstrained())
      throw std::invalid_argument(
          "CMC tangent inequalities must be unary, homogeneous, and "
          "soft-scaled");
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

const Matrix& IEManifoldLinearization::rows(size_t index, bool whitened) const {
  const auto& block = blocks_.at(index);
  return whitened ? block.whitened : block.raw;
}

Matrix IEManifoldLinearization::workingRows(const IndexSet& indices,
                                            bool whitened) const {
  size_t count = 0;
  for (size_t index : indices) count += rows(index).rows();
  const size_t dimension =
      blocks_.empty() ? 0 : blocks_.begin()->second.raw.cols();
  Matrix result(count, dimension);
  size_t offset = 0;
  for (size_t index : indices) {
    const auto& blockRows = rows(index, whitened);
    result.middleRows(offset, blockRows.rows()) = blockRows;
    offset += blockRows.rows();
  }
  return result;
}

IndexSet IEManifoldLinearization::violated(const Vector& delta,
                                           double tolerance) const {
  IndexSet result;
  for (const auto& [index, block] : blocks_)
    if (block.raw.rows() && (block.raw * delta).maxCoeff() > tolerance)
      result.insert(index);
  return result;
}

bool IEManifoldLinearization::active(const Vector& delta,
                                     const IndexSet& indices,
                                     double tolerance) const {
  for (size_t index : indices)
    if ((rows(index, true) * delta).norm() > tolerance) return false;
  return true;
}

std::pair<IndexSet, Vector> IEManifoldLinearization::project(
    const Vector& delta) const {
  auto result = cone_->project(delta);
  const std::vector<size_t> original(indices_.begin(), indices_.end());
  IndexSet indices;
  for (size_t index : result.first) indices.insert(original.at(index));
  return {indices, std::move(result.second)};
}

LinearIConstraintMap IEManifoldLinearization::tangentConstraints(
    Key componentKey) const {
  LinearIConstraintMap result;
  for (const auto& [index, block] : blocks_)
    result.emplace(
        index, std::make_shared<JacobianLinearInequalityConstraint>(
                   std::make_shared<JacobianFactor>(
                       componentKey, block.raw, Vector::Zero(block.raw.rows()),
                       block.noise)));
  return result;
}

LinearIConstraintMap IEManifoldLinearization::ambientConstraints() const {
  LinearIConstraintMap result;
  for (const auto& [index, block] : blocks_)
    if (block.ambient)
      result.emplace(index,
                     std::make_shared<JacobianLinearInequalityConstraint>(
                         std::make_shared<JacobianFactor>(*block.ambient)));
  return result;
}

IndexSet IEManifoldLinearization::blockingIndices(
    const VectorValues& direction) const {
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
