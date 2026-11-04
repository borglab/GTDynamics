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

#include <Eigen/LU>
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
internal::WorkingFace IEManifoldLinearization::workingFace(
    const IndexSet& selected) const {
  return internal::WorkingFace(workingRows(selected));
}
internal::WorkingFace IEManifoldLinearization::workingFace(
    const RowSet& selected) const {
  Matrix matrix(selected.size(), blocks_.begin()->second.raw.cols());
  size_t i = 0;
  for (const auto& [block, row] : selected)
    matrix.row(i++) = rows(block, true).row(row);
  return internal::WorkingFace(std::move(matrix));
}
IEManifoldLinearization::RowSet IEManifoldLinearization::initialRows(
    const IndexSet& selected, const Vector& initial) const {
  RowSet active;
  for (const auto& [index, block] : blocks_)
    for (Eigen::Index row = 0; row < block.whitened.rows(); ++row) {
      const double value = block.whitened.row(row).dot(initial);
      if (value > 1e-7 || (selected.exists(index) && std::abs(value) > 1e-7))
        throw std::invalid_argument("CMC QP warm start is not feasible");
      if (selected.exists(index)) active.emplace(index, row);
    }
  return active;
}
IndexSet IEManifoldLinearization::completeBlocks(const RowSet& active) const {
  IndexSet result;
  for (const auto& [index, block] : blocks_) {
    bool complete = block.raw.rows() > 0;
    for (Eigen::Index row = 0; row < block.raw.rows(); ++row)
      complete &= active.count({index, row}) != 0;
    if (complete) result.insert(index);
  }
  return result;
}
std::pair<double, std::optional<IEManifoldLinearization::Row>>
IEManifoldLinearization::boundaryStep(const Vector& current,
                                      const Vector& direction,
                                      const RowSet& active,
                                      double maximumStep) const {
  double step = maximumStep;
  std::optional<Row> entering;
  for (const auto& [index, block] : blocks_)
    for (Eigen::Index row = 0; row < block.whitened.rows(); ++row) {
      if (active.count({index, row})) continue;
      const double slope = block.whitened.row(row).dot(direction);
      if (slope <= 0.0) continue;
      const double candidate = -block.whitened.row(row).dot(current) / slope;
      if (candidate < step) {
        step = std::max(0.0, candidate);
        entering = Row{index, row};
      }
    }
  return {step, entering};
}
std::pair<double, std::optional<IEManifoldLinearization::Row>>
IEManifoldLinearization::leavingRow(const Vector& gradient,
                                    const RowSet& active) const {
  if (active.empty()) return {-1e-7, std::nullopt};
  Matrix matrix(active.size(), gradient.size());
  size_t i = 0;
  for (const auto& [index, row] : active)
    matrix.row(i++) = rows(index, true).row(row);
  const Matrix transpose = matrix.transpose();
  const Vector multipliers = transpose.fullPivLu().solve(-gradient);
  if (!multipliers.allFinite() || !(transpose * multipliers + gradient)
                                       .isZero(1e-6 * (1.0 + gradient.norm())))
    throw std::runtime_error("CMC local dual equations are not stationary");
  double mostNegative = -1e-7;
  std::optional<Row> leaving;
  i = 0;
  for (const auto& row : active) {
    if (multipliers(i) < mostNegative) {
      mostNegative = multipliers(i);
      leaving = row;
    }
    ++i;
  }
  return {mostNegative, leaving};
}
}  // namespace gtdynamics
