/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TangentSpaceMetric.cpp
 * @brief Construct and apply a whitened tangent-space projection metric.
 * @author Frank Dellaert
 */

#include <Eigen/Cholesky>
#include <gtdynamics/cmopt/TangentSpaceMetric.h>
#include <stdexcept>

namespace gtdynamics {

TangentSpaceMetric::TangentSpaceMetric(const TangentSpaceBasis &basis,
                                       const Values &values,
                                       const VectorValues &precisions,
                                       bool whiten)
    : precisions_(precisions), dim_(basis.dim()), whiten_(whiten),
      cost_jacobian_(Matrix::Identity(dim_, dim_)) {
  Matrix metric = Matrix::Zero(dim_, dim_);
  for (const auto &kv : values) {
    Matrix B =
        dim_ ? basis.recoverJacobian(kv.key) : Matrix::Zero(kv.value.dim(), 0);
    if (B.rows() != kv.value.dim() || B.cols() != dim_ || !B.allFinite())
      throw std::invalid_argument("Invalid tangent recovery basis");
    maps_.emplace(kv.key, std::move(B));
  }
  for (const auto &[key, weights] : precisions_) {
    const Matrix &B = maps_.at(key);
    if (weights.size() != B.rows() || !weights.allFinite() ||
        (weights.array() <= 0).any())
      throw std::invalid_argument("Invalid projection metric precision");
    metric.noalias() += B.transpose() * weights.asDiagonal() * B;
  }
  if (!metric.allFinite())
    throw std::invalid_argument("Nonfinite reduced projection metric");
  if (!dim_)
    return;
  Eigen::LLT<Matrix> cholesky(metric);
  if (cholesky.info() != Eigen::Success)
    throw std::invalid_argument("Projection metric must be positive definite");
  if (!whiten_) {
    cost_jacobian_ = cholesky.matrixU();
    return;
  }
  const Matrix inverseR =
      cholesky.matrixU().solve(Matrix::Identity(dim_, dim_));
  for (auto &[key, map] : maps_) {
    map = (map * inverseR).eval();
    if (!map.allFinite())
      throw std::invalid_argument("Nonfinite whitened recovery basis");
  }
}

bool TangentSpaceMetric::matches(const VectorValues &precisions,
                                 bool whiten) const {
  if (whiten != whiten_)
    return false;
  if (precisions.size() != precisions_.size())
    return false;
  for (const auto &[key, weights] : precisions) {
    if (!precisions_.exists(key))
      return false;
    const Vector &previous = precisions_.at(key);
    if (weights.size() != previous.size() ||
        !(weights.array() == previous.array()).all())
      return false;
  }
  return true;
}

Vector TangentSpaceMetric::project(const VectorValues &delta) const {
  Vector target = Vector::Zero(dim_);
  for (const auto &[key, weights] : precisions_) {
    if (delta.exists(key))
      target.noalias() +=
          maps_.at(key).transpose() * weights.cwiseProduct(delta.at(key));
  }
  if (!whiten_ && dim_) {
    const Vector rhs =
        cost_jacobian_.transpose().triangularView<Eigen::Lower>().solve(target);
    target = cost_jacobian_.triangularView<Eigen::Upper>().solve(rhs);
  }
  if (!target.allFinite())
    throw std::invalid_argument("Nonfinite reduced projection target");
  return target;
}

VectorValues TangentSpaceMetric::recover(const Vector &u) const {
  VectorValues delta;
  if (u.size() != dim_ || !u.allFinite())
    throw std::invalid_argument("Invalid projection coordinates");
  for (const auto &[key, map] : maps_) {
    const Vector step = map * u;
    if (!step.allFinite())
      throw std::invalid_argument("Nonfinite recovered projection step");
    delta.insert(key, step);
  }
  return delta;
}

} // namespace gtdynamics
