/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file AffineBasisCounter.h
 * @brief Count type-owned equality preparation without changing its algorithms.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/constrained/TangentSpaceBasis.h>

#include <atomic>

namespace gtdynamics::internal {
struct AffineBasisCounts {
  std::atomic<size_t> preparations{0}, numerical{0};
};

// Only the unconstructed template is wrapped; numerical bases are unchanged.
class CountedBasisTemplate final : public TangentSpaceBasis {
  std::shared_ptr<const TangentSpaceBasis> source_;
  std::shared_ptr<AffineBasisCounts> counts_;

 public:
  CountedBasisTemplate(std::shared_ptr<const TangentSpaceBasis> source,
                       std::shared_ptr<AffineBasisCounts> counts)
      : source_(std::move(source)), counts_(std::move(counts)) {}

  shared_ptr createWithNewValues(const Values& values) const override {
    ++counts_->numerical;
    return source_->createWithNewValues(values);
  }
  shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints& constraints, const Values& values,
      bool fromScratch = false) const override {
    return source_->createWithAdditionalConstraints(constraints, values,
                                                    fromScratch);
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
    throw std::logic_error(
        "Construct a fresh numerical basis from the template");
  }
  void print(const KeyFormatter& = DefaultKeyFormatter) override {
    std::cout << "Equality template dimension: " << dim() << '\n';
  }
};

class CountedAffineBasisCreator final : public TangentSpaceBasisCreator {
  TangentSpaceBasisCreator::shared_ptr source_;
  std::shared_ptr<AffineBasisCounts> counts_;

 public:
  CountedAffineBasisCreator(TangentSpaceBasisCreator::shared_ptr source,
                            std::shared_ptr<AffineBasisCounts> counts)
      : TangentSpaceBasisCreator(*source),
        source_(std::move(source)),
        counts_(std::move(counts)) {}
  shared_ptr clone() const override {
    return std::make_shared<CountedAffineBasisCreator>(source_->clone(),
                                                       counts_);
  }
  std::shared_ptr<const TangentSpaceBasis> prepare(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const std::map<Key, size_t>& dimensions) const override {
    ++counts_->preparations;
    return std::make_shared<CountedBasisTemplate>(
        source_->prepare(constraints, dimensions), counts_);
  }
  TangentSpaceBasis::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const Values& values) const override {
    return source_->create(constraints, values);
  }
};
}  // namespace gtdynamics::internal
