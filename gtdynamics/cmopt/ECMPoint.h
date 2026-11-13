/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file ECMPoint.h
 * @brief Concrete equality-only GTSAM point value.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once
#include <gtdynamics/cmopt/EConstraintManifold.h>
#include <gtdynamics/cmcopt/IECMPoint.h>

namespace gtdynamics {
class ECMPoint : public IECMPoint {
 public:
  using shared_ptr = std::shared_ptr<ECMPoint>;
  ECMPoint() = default;
  EConstraintManifold::shared_ptr manifold() const {
    return std::static_pointer_cast<const EConstraintManifold>(manifold_);
  }
  // An explicit checked boundary; ordinary upcasting never removes
  // inequalities.
  static ECMPoint fromIEPoint(const IECMPoint& point);
  ECMPoint retract(const Vector& delta) const;
  ECMPoint retract(const Vector& delta, const std::optional<IndexSet>& blocking,
                   IERetractionInfo* info = nullptr,
                   const VectorValues* metricSigmas = nullptr) const;
  ECMPoint retract(const VectorValues& delta,
                   const std::optional<IndexSet>& blocking = {},
                   IERetractionInfo* info = nullptr,
                   const VectorValues* metricSigmas = nullptr) const;
  Vector localCoordinates(const ECMPoint& other) const {
    return IECMPoint::localCoordinates(other);
  }
  ECMPoint equalityView() const { return *this; }

 private:
  void validateAssignment(const CMPoint& other) const override final;
  friend class EConstraintManifold;
  explicit ECMPoint(const IECMPoint& point) : IECMPoint(point) {}
};
using ECMPoints = std::map<Key, ECMPoint>;
Values ambientValues(const ECMPoints& points);
KeyVector pointKeys(const ECMPoints& points);
IECMPoints widenPoints(const ECMPoints& points);
ECMPoints extractECMPoints(const Values& values);
VectorValues computeTangentVector(const ECMPoints& points,
                                  const VectorValues& delta);
ECMPoints retractPoints(const ECMPoints& points, const VectorValues& delta);
}  // namespace gtdynamics
namespace gtsam {
template <>
struct traits<gtdynamics::ECMPoint> : internal::Manifold<gtdynamics::ECMPoint> {
};
}  // namespace gtsam
