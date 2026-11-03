/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file TangentConeLinearization.cpp
 * @brief Select local working faces around unconstrained Gaussian solves.
 * @author GTDynamics contributors
 */
#include <gtdynamics/cmcopt/TangentConeLinearization.h>

#include <Eigen/LU>
#include <algorithm>
#include <stdexcept>

namespace gtdynamics {
namespace internal {
namespace {
using namespace gtsam;

// Working sets contain only a few rows per component. Appending here never
// constructs a trajectory-wide dense matrix or a constrained Gaussian factor.
void appendRows(std::map<Key, Matrix>& blocks, Key key, const Matrix& rows) {
  auto [entry, inserted] = blocks.emplace(key, rows);
  if (inserted) return;
  Matrix& existing = entry->second;
  const auto offset = existing.rows();
  existing.conservativeResize(offset + rows.rows(), existing.cols());
  existing.bottomRows(rows.rows()) = rows;
}

struct FaceRow {
  Key key;
  Eigen::RowVectorXd row;
};

// The global gradient already includes cross-component cost terms. Solving
// A_i^T lambda_i = -gradient_i is then local. Full pivoting assigns zero to
// free dual coordinates when faces are dependent. A negative multiplier asks
// to release that row; the next boundary step can activate another row.
size_t leavingFace(const std::vector<FaceRow>& rows,
                   const std::vector<bool>& active,
                   const VectorValues& gradient) {
  std::map<Key, Matrix> blocks;
  std::map<Key, std::vector<size_t>> indices;
  for (size_t i = 0; i < rows.size(); ++i) {
    if (!active[i]) continue;
    appendRows(blocks, rows[i].key, rows[i].row);
    indices[rows[i].key].push_back(i);
  }
  size_t leaving = rows.size();
  double mostNegative = -1e-7;
  for (const auto& [key, block] : blocks) {
    const Matrix transpose = block.transpose();
    const Vector multipliers = transpose.fullPivLu().solve(-gradient.at(key));
    if (!multipliers.allFinite() ||
        !(transpose * multipliers + gradient.at(key))
             .isZero(1e-6 * (1.0 + gradient.at(key).norm())))
      throw std::runtime_error("CMC local dual equations are not stationary");
    for (size_t i = 0; i < indices.at(key).size(); ++i) {
      if (multipliers(i) < mostNegative) {
        mostNegative = multipliers(i);
        leaving = indices.at(key)[i];
      }
    }
  }
  return leaving;
}
}  // namespace

TangentConeLinearization::TangentConeLinearization(
    const LinearInequalityConstraints& constraints) {
  for (const auto& constraint : constraints) {
    const auto factor = constraint->createL2Factor();
    if (factor->size() != 1 || !factor->getb().isZero(0.0) ||
        factor->isConstrained())
      throw std::invalid_argument(
          "CMC tangent inequalities must be unary, homogeneous, and "
          "soft-scaled");
    const auto whitened = factor->whiten();
    Block block{factor->front(), factor->getA(factor->begin()),
                whitened.getA(whitened.begin())};
    if (!block.rows.allFinite() || !block.whitenedRows.allFinite())
      throw std::invalid_argument("CMC tangent inequality has nonfinite rows");
    blocks_.push_back(std::move(block));
  }
}

ComponentTangentSpace TangentConeLinearization::workingSpace(
    const IndexSet& blockingIndices) const {
  std::map<gtsam::Key, gtsam::Matrix> faces;
  for (size_t index : blockingIndices) {
    const auto& block = blocks_.at(index);
    appendRows(faces, block.key, block.rows);
  }
  return ComponentTangentSpace(faces);
}

IndexSet TangentConeLinearization::violated(const gtsam::VectorValues& delta,
                                            double tolerance) const {
  IndexSet indices;
  for (size_t index = 0; index < blocks_.size(); ++index) {
    const auto& block = blocks_[index];
    if (block.rows.rows() &&
        (block.rows * delta.at(block.key)).maxCoeff() > tolerance)
      indices.insert(index);
  }
  return indices;
}

bool TangentConeLinearization::active(const gtsam::VectorValues& delta,
                                      const IndexSet& indices,
                                      double tolerance) const {
  for (size_t index : indices) {
    const auto& block = blocks_.at(index);
    if ((block.whitenedRows * delta.at(block.key)).norm() > tolerance)
      return false;
  }
  return true;
}

TangentConeLinearization::Result TangentConeLinearization::refine(
    const gtsam::GaussianFactorGraph& cost, const gtsam::VectorValues& initial,
    const IndexSet& blockingIndices,
    const gtsam::NonlinearOptimizerParams& params, size_t maxIterations) const {
  using namespace gtsam;
  std::vector<FaceRow> rows;
  std::vector<bool> active;
  for (size_t index = 0; index < blocks_.size(); ++index) {
    const auto& block = blocks_[index];
    for (Eigen::Index row = 0; row < block.rows.rows(); ++row) {
      rows.push_back({block.key, block.whitenedRows.row(row)});
      active.push_back(blockingIndices.exists(index));
      const double value = rows.back().row.dot(initial.at(block.key));
      if (value > 1e-7 || (active.back() && std::abs(value) > 1e-7))
        throw std::invalid_argument("CMC QP warm start is not feasible");
    }
  }

  Result result{initial, {}, 0, false};
  for (size_t iteration = 0; iteration < maxIterations; ++iteration) {
    std::map<Key, Matrix> faces;
    for (size_t i = 0; i < rows.size(); ++i)
      if (active[i]) appendRows(faces, rows[i].key, rows[i].row);
    const VectorValues candidate =
        ComponentTangentSpace(faces).solve(cost, params);
    ++result.numSolves;
    const VectorValues direction = candidate - result.delta;

    // At a working-set minimum, positive contact multipliers support the
    // boundary. Negative multipliers indicate a face that must be released.
    if (candidate.equals(result.delta, 1e-7)) {
      const size_t leaving =
          leavingFace(rows, active, cost.gradient(candidate));
      result.delta = candidate;
      if (leaving == rows.size()) {
        result.converged = true;
        break;
      }
      active[leaving] = false;
      continue;
    }

    // Advance to the quadratic minimum or the first new boundary. All rows
    // are homogeneous in the current tangent coordinates, so zero is feasible.
    double step = 1.0;
    size_t entering = rows.size();
    for (size_t i = 0; i < rows.size(); ++i) {
      if (active[i]) continue;
      const auto& row = rows[i];
      const double slope = row.row.dot(direction.at(row.key));
      if (slope <= 0.0) continue;
      const double candidateStep =
          -row.row.dot(result.delta.at(row.key)) / slope;
      if (candidateStep < step) {
        step = std::max(0.0, candidateStep);
        entering = i;
      }
    }
    result.delta += step * direction;
    if (entering < rows.size()) active[entering] = true;
  }

  // Retraction promotes whole blocks only. A partially active multirow block
  // must retain its inequality semantics at the nonlinear level.
  size_t offset = 0;
  for (size_t index = 0; index < blocks_.size(); ++index) {
    const size_t count = blocks_[index].rows.rows();
    if (count &&
        std::all_of(active.begin() + offset, active.begin() + offset + count,
                    [](bool value) { return value; }))
      result.blockingIndices.insert(index);
    offset += count;
  }
  if (!violated(result.delta, 1e-7).empty()) result.converged = false;
  return result;
}
}  // namespace internal
}  // namespace gtdynamics
