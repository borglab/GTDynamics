/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ReadOnlyBasis.h
 * @brief Protect shared numerical geometry behind legacy mutable interfaces.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/cmopt/TangentSpaceBasis.h>

namespace gtdynamics::internal {

/** Forward chart operations without exposing a mutable alias to shared data. */
class ReadOnlyBasis : public TangentSpaceBasis {
 protected:
  std::shared_ptr<const TangentSpaceBasis> source_;

  static std::shared_ptr<const TangentSpaceBasis> constructed(
      const shared_ptr& basis, const Values& values) {
    if (!basis->isConstructed()) basis->construct(values);
    return basis;
  }

 public:
  explicit ReadOnlyBasis(std::shared_ptr<const TangentSpaceBasis> source)
      : source_(std::move(source)) {
    if (!source_ || !source_->isConstructed())
      throw std::invalid_argument("Read-only basis requires complete geometry");
    is_constructed_ = true;
  }

  shared_ptr createWithNewValues(const Values& values) const override {
    return std::make_shared<ReadOnlyBasis>(
        constructed(source_->createWithNewValues(values), values));
  }

  shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints& constraints, const Values& values,
      bool fromScratch = false) const override {
    return std::make_shared<ReadOnlyBasis>(
        constructed(source_->createWithAdditionalConstraints(
                        constraints, values, fromScratch),
                    values));
  }

  VectorValues computeTangentVector(const Vector& xi) const override {
    return source_->computeTangentVector(xi);
  }
  Vector computeXi(const VectorValues& delta) const override {
    return source_->computeXi(delta);
  }
  Matrix recoverJacobian(const Key& key) const override {
    return source_->recoverJacobian(key);
  }
  Vector localCoordinates(const Values& a, const Values& b) const override {
    return source_->localCoordinates(a, b);
  }
  size_t dim() const override { return source_->dim(); }

  void construct(const Values&) override {
    throw std::logic_error("Cannot reconstruct a read-only basis view");
  }
  void print(const KeyFormatter& formatter = DefaultKeyFormatter) override {
    for (const auto& vector : basisVectors()) vector.print("Basis", formatter);
  }
};

}  // namespace gtdynamics::internal
