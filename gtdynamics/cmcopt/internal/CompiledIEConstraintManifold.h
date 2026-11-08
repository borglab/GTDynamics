/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CompiledIEConstraintManifold.h
 * @brief Adapt ordinary GTSAM manifold types to CMC ambient recovery.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/CMPoint.h>

#include <Eigen/QR>

namespace gtdynamics::internal {
/** One native boundary block, g <= 0, with raw local-slot derivatives. */
struct NativeBoundary {
  Vector expression;
  Vector sigmas;
  std::map<Key, Matrix> jacobians;
};

/** Small compiled charts supply their recovery Jacobian blocks directly.
 * This stores component-local blocks only, never a global trajectory basis.
 */
class NativeBasis final : public TangentSpaceBasis {
  std::map<Key, Matrix> blocks_;
  size_t dimension_;

 public:
  /** Capture recovery blocks in ambient-key coordinates. */
  NativeBasis(const std::map<Key, Matrix>& blocks, size_t dimension)
      : blocks_(blocks), dimension_(dimension) {
    is_constructed_ = true;
  }

  /// Native adapters rebuild geometry through their own linearize operation.
  shared_ptr createWithNewValues(const Values&) const override {
    throw std::logic_error("Native geometry must be rebuilt by its type");
  }

  /// A native type owns its boundary operation, rather than legacy face bases.
  shared_ptr createWithAdditionalConstraints(
      const NonlinearEqualityConstraints&, const Values&,
      bool = false) const override {
    throw std::logic_error("Native boundary geometry belongs to its type");
  }

  /// Lift a vector through the supplied recovery blocks.
  VectorValues computeTangentVector(const Vector& xi) const override {
    VectorValues result;
    for (const auto& [key, block] : blocks_) result.insert(key, block * xi);
    return result;
  }

  /// Invert the component-local full-column-rank recovery map.
  Vector computeXi(const VectorValues& delta) const override {
    if (!dimension_) return Vector(0);
    size_t rows = 0;
    for (const auto& [key, block] : blocks_) rows += block.rows();

    // Stack component-local recovery blocks to invert the ambient embedding.
    Matrix matrix(rows, dimension_);
    Vector rhs(rows);
    size_t offset = 0;
    for (const auto& [key, block] : blocks_) {
      matrix.middleRows(offset, block.rows()) = block;
      rhs.segment(offset, block.rows()) = delta.at(key);
      offset += block.rows();
    }
    return matrix.colPivHouseholderQr().solve(rhs);
  }

  /// Derivative of recovering one ambient variable.
  Matrix recoverJacobian(const Key& key) const override {
    return blocks_.at(key);
  }

  /// Native point operations normally delegate Local directly to gtsam::traits.
  Vector localCoordinates(const Values& a, const Values& b) const override {
    return computeXi(a.localCoordinates(b));
  }

  /// Intrinsic compiled chart dimension.
  size_t dim() const override { return dimension_; }
  /// Recovery blocks are already constructed and immutable.
  void construct(const Values&) override {}
  /// Display recovery blocks for diagnostics.
  void print(const KeyFormatter& formatter = DefaultKeyFormatter) override {
    computeTangentVector(Vector::Zero(dimension_))
        .print("Native chart", formatter);
  }
};

/** T conforms to GTSAM's manifold concept. Recovery supplies:
 * fromAmbient(Values), ambient(T), jacobians(T), boundaries(T), and
 * boundary(T, IndexSet), all using type-local slots. The adapter invokes no
 * factor-defined fallback and constructs no fictitious constraint factors.
 * Conditioning of native types belongs in their immutable Recovery parameters.
 */
template <class T, class Recovery>
class CompiledIEConstraintManifold final : public IEConstraintManifold {
  Recovery recovery_;
  Values local(const CMPoint& point) const {
    Values result;
    for (const auto& [slot, key] : point.bindings())
      result.insert(slot, point.values().at(key));
    return result;
  }

  Values ambient(const T& value, const CMPoint& point) const {
    Values result;
    const auto slots = recovery_.ambient(value);
    for (const auto& [slot, key] : point.bindings())
      result.insert(key, slots.at(slot));
    return result;
  }

 public:
  /** Capture the compiled type's local embedding and immutable parameters. */
  CompiledIEConstraintManifold(const T& prototype, const Recovery& recovery)
      : IEConstraintManifold(recovery.ambient(prototype),
                             traits<T>::GetDimension(prototype)),
        recovery_(recovery) {}

 protected:
  /// Construct numerical recovery and active-boundary geometry at this point.
  std::shared_ptr<const CMTangentSpace> linearizeAt(
      const CMPoint& point, const CMTangentSpace* = nullptr) const override {
    const auto values = local(point);
    const T native = recovery_.fromAmbient(values);
    if (!values.equals(recovery_.ambient(native), 1e-8))
      throw std::invalid_argument(
          "Ambient state is not a value of the compiled type");

    // Bind native recovery blocks to the point's trajectory-specific keys.
    std::map<Key, Matrix> blocks;
    for (const auto& [slot, block] : recovery_.jacobians(native))
      blocks.emplace(point.bindings().at(slot), block);
    auto basis = std::make_shared<NativeBasis>(blocks, dim());

    // Boundary blocks follow the same reduced-row convention as factor types.
    const auto boundaries = recovery_.boundaries(native);
    LinearIConstraintMap tangent, ambientRows;
    for (size_t index : point.activeIndices()) {
      const auto& boundary = boundaries.at(index);
      Matrix reduced = Matrix::Zero(boundary.expression.size(), dim());
      std::vector<std::pair<Key, Matrix>> terms;
      for (const auto& [slot, jacobian] : boundary.jacobians) {
        Key key = point.bindings().at(slot);
        reduced += jacobian * blocks.at(key);
        terms.emplace_back(key, jacobian);
      }
      auto noise = noiseModel::Diagonal::Sigmas(boundary.sigmas);
      Vector zero = Vector::Zero(reduced.rows());
      tangent.emplace(
          index,
          std::make_shared<JacobianLinearInequalityConstraint>(
              std::make_shared<JacobianFactor>(1, reduced, zero, noise)));
      ambientRows.emplace(
          index, std::make_shared<JacobianLinearInequalityConstraint>(
                     std::make_shared<JacobianFactor>(terms, zero, noise)));
    }
    return std::make_shared<CMTangentSpace>(basis, tangent, ambientRows);
  }

 public:
  /// Delegate ordinary vector retraction to the compiled GTSAM traits.
  CMPoint retract(const CMPoint& point, const Vector& delta,
                  const std::optional<IndexSet>& blocking = {},
                  IERetractionInfo* = nullptr,
                  const VectorValues* = nullptr) const override {
    typename traits<T>::TangentVector v = delta;
    T next = traits<T>::Retract(recovery_.fromAmbient(local(point)), v);

    // The compiled type owns explicit movement onto a blocking face.
    if (blocking && !blocking->empty())
      next = recovery_.boundary(next, *blocking);
    return point.createWithNewValues(ambient(next, point));
  }

  /// Lifted updates use the same native chart, never a factor-graph retractor.
  CMPoint retractAmbient(const CMPoint& point, const VectorValues& delta,
                         const std::optional<IndexSet>& blocking = {},
                         IERetractionInfo* info = nullptr,
                         const VectorValues* metric = nullptr) const override {
    return retract(point, point.toCoordinates(delta), blocking, info, metric);
  }

  /// Delegate chart differences to the compiled GTSAM traits.
  Vector localCoordinates(const CMPoint& a, const CMPoint& b) const override {
    return traits<T>::Local(recovery_.fromAmbient(local(a)),
                            recovery_.fromAmbient(local(b)));
  }

  /// Select tight native boundaries, preserving complete block indices.
  IndexSet identifyActive(
      const CMPoint& point,
      const std::optional<IndexSet>& supplied = {}) const override {
    const auto boundaries =
        recovery_.boundaries(recovery_.fromAmbient(local(point)));
    if (supplied) {
      for (size_t index : *supplied)
        if (index >= boundaries.size())
          throw std::invalid_argument("Invalid native boundary index");
      return *supplied;
    }
    IndexSet active;
    for (size_t i = 0; i < boundaries.size(); ++i)
      if ((boundaries[i].expression.array() >= 0).any()) active.insert(i);
    return active;
  }

  /// Native block counts are independent of factor storage.
  size_t inequalityCount(const CMPoint& point) const override {
    return recovery_.boundaries(recovery_.fromAmbient(local(point))).size();
  }

  /// Return the raw native inequality expression.
  Vector inequalityResidual(const CMPoint& point, size_t index) const override {
    return recovery_.boundaries(recovery_.fromAmbient(local(point)))
        .at(index)
        .expression;
  }

  /// Measure embedding consistency without invented equality factors.
  double equalityViolation(const CMPoint& point) const override {
    const auto state = local(point);
    return state
        .localCoordinates(recovery_.ambient(recovery_.fromAmbient(state)))
        .norm();
  }

  /// Measure positive normalized boundary residuals.
  double inequalityViolation(const CMPoint& point) const override {
    double squared = 0;
    for (const auto& block :
         recovery_.boundaries(recovery_.fromAmbient(local(point))))
      squared += block.expression.cwiseMax(0)
                     .cwiseQuotient(block.sigmas)
                     .squaredNorm();
    return std::sqrt(squared);
  }

  /// Native boundary movement uses the explicit recovery operation.
  CMPoint moveToBoundary(const CMPoint& point, const IndexSet& active,
                         IERetractionInfo* = nullptr,
                         const VectorValues* = nullptr) const override {
    return point.createWithNewValues(
        ambient(recovery_.boundary(recovery_.fromAmbient(local(point)), active),
                point),
        active);
  }
};

}  // namespace gtdynamics::internal
