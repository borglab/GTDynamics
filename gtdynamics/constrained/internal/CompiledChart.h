/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CompiledChart.h
 * @brief Shared GTSAM chart and recovery support for runtime types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IECMPoint.h>

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
template <class T, class Recovery, class Base = IEConstraintManifold>
class CompiledChart : public Base {
 protected:
  Recovery recovery_;
  Values local(const IECMPoint& point) const {
    Values result;
    for (const auto& [slot, key] : point.bindings())
      result.insert(slot, point.values().at(key));
    return result;
  }

  Values ambient(const T& value, const IECMPoint& point) const {
    Values result;
    const auto slots = recovery_.ambient(value);
    for (const auto& [slot, key] : point.bindings())
      result.insert(key, slots.at(slot));
    return result;
  }

 public:
  /** Capture the compiled type's local embedding and immutable parameters. */
  CompiledChart(const T& prototype, const Recovery& recovery)
      : Base(recovery.ambient(prototype), traits<T>::GetDimension(prototype)),
        recovery_(recovery) {}

 protected:
  // Equality charts require no boundary methods on their Recovery object.
  virtual std::vector<NativeBoundary> nativeBoundaries(const T&) const {
    return {};
  }
  virtual T applyBoundary(const T& value, const IndexSet& active) const {
    if (!active.empty())
      throw std::invalid_argument("Equality charts have no boundary blocks");
    return value;
  }

  /// Construct numerical recovery and active-boundary geometry at this point.
  std::shared_ptr<const CMTangentSpace> linearizeAt(
      const IECMPoint& point) const override {
    const auto values = local(point);
    const T native = recovery_.fromAmbient(values);
    if (!values.equals(recovery_.ambient(native), 1e-8))
      throw std::invalid_argument(
          "Ambient state is not a value of the compiled type");

    // Bind native recovery blocks to the point's trajectory-specific keys.
    std::map<Key, Matrix> blocks;
    for (const auto& [slot, block] : recovery_.jacobians(native))
      blocks.emplace(point.bindings().at(slot), block);
    auto basis = std::make_shared<NativeBasis>(blocks, this->dim());

    // Boundary blocks follow the same reduced-row convention as factor types.
    const auto boundaries = nativeBoundaries(native);
    LinearIConstraintMap tangent, ambientRows;
    for (size_t index : point.activeIndices()) {
      const auto& boundary = boundaries.at(index);
      Matrix reduced = Matrix::Zero(boundary.expression.size(), this->dim());
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
  IECMPoint retract(const IECMPoint& point, const Vector& delta,
                    const std::optional<IndexSet>& blocking = {},
                    IERetractionInfo* = nullptr,
                    const VectorValues* = nullptr) const override {
    this->validatePointType(point);
    typename traits<T>::TangentVector v = delta;
    T next = traits<T>::Retract(recovery_.fromAmbient(local(point)), v);

    // The compiled type owns explicit movement onto a blocking face.
    if (blocking && !blocking->empty()) next = applyBoundary(next, *blocking);
    return point.manifold()->point(point.bindings(), ambient(next, point));
  }

  /// Lifted updates use the same native chart, never a factor-graph retractor.
  IECMPoint retractAmbient(
      const IECMPoint& point, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr,
      const VectorValues* metric = nullptr) const override {
    this->validatePointType(point);
    return retract(point, point.toCoordinates(delta), blocking, info, metric);
  }

  /// Delegate chart differences to the compiled GTSAM traits.
  Vector localCoordinates(const IECMPoint& a,
                          const IECMPoint& b) const override {
    this->validatePointType(a);
    this->validatePointType(b);
    if (a.bindings() != b.bindings())
      throw std::invalid_argument(
          "Local coordinates require identical bindings");
    return traits<T>::Local(recovery_.fromAmbient(local(a)),
                            recovery_.fromAmbient(local(b)));
  }

  /// Measure embedding consistency without invented equality factors.
  double equalityViolation(const IECMPoint& point) const override {
    const auto state = local(point);
    return state
        .localCoordinates(recovery_.ambient(recovery_.fromAmbient(state)))
        .norm();
  }

  /// Native boundary movement uses the explicit recovery operation.
  IECMPoint moveToBoundary(const IECMPoint& point, const IndexSet& active,
                           IERetractionInfo* = nullptr,
                           const VectorValues* = nullptr) const override {
    this->validatePointType(point);
    return point.manifold()->point(
        point.bindings(),
        ambient(applyBoundary(recovery_.fromAmbient(local(point)), active),
                point),
        active);
  }
};

}  // namespace gtdynamics::internal
