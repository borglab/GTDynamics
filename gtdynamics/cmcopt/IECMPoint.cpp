/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IECMPoint.cpp
 * @brief Values and GTSAM chart operations for runtime manifold types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/IECMPoint.h>
#include <gtdynamics/cmopt/ECMPoint.h>


namespace gtdynamics {
ECMPoint IECMPoint::equalityView() const {
  if (std::dynamic_pointer_cast<const EConstraintManifold>(manifold()))
    return ECMPoint::fromIEPoint(*this);
  return manifold()->equalityView()->point(bindings(), values());
}

IECMPoint IECMPoint::retract(const Vector& delta) const {
  return retract(delta, {});
}

IECMPoint IECMPoint::retract(const Vector& delta,
                         const std::optional<IndexSet>& blocking,
                         IERetractionInfo* info,
                         const VectorValues* metricSigmas) const {
  if (!manifold_)
    throw std::invalid_argument("Cannot retract an unbound IECMPoint");
  if (delta.size() != dim())
    throw std::invalid_argument("Incorrect IECMPoint tangent dimension");
  return manifold()->retract(*this, delta, blocking, info, metricSigmas);
}

IECMPoint IECMPoint::retract(const VectorValues& delta,
                         const std::optional<IndexSet>& blocking,
                         IERetractionInfo* info,
                         const VectorValues* metricSigmas) const {
  if (!manifold_)
    throw std::invalid_argument("Cannot retract an unbound IECMPoint");
  return manifold()->retractAmbient(*this, delta, blocking, info, metricSigmas);
}

Vector IECMPoint::localCoordinates(const IECMPoint& other) const {
  if (!manifold_ || manifold_ != other.manifold_ ||
      bindings() != other.bindings())
    throw std::invalid_argument(
        "IECMPoint local coordinates require the same type and bindings");
  return manifold()->localCoordinates(*this, other);
}

std::pair<IndexSet, Vector> IECMPoint::projectTangentCone(
    const Vector& delta) const {
  return tangentSpace_->project(delta);
}

std::pair<IndexSet, VectorValues> IECMPoint::projectTangentCone(
    const VectorValues& delta) const {
  auto [active, reduced] = projectTangentCone(toCoordinates(delta));
  return {active, toAmbient(reduced)};
}

IndexSet IECMPoint::blockingIndices(const VectorValues& delta) const {
  return tangentSpace_->blockingIndices(delta);
}

IECMPoint IECMPoint::moveToBoundary(const IndexSet& active, IERetractionInfo* info,
                                const VectorValues* metricSigmas) const {
  return manifold()->moveToBoundary(*this, active, info, metricSigmas);
}

Vector IECMPoint::inequalityResidual(size_t index) const {
  return manifold()->inequalityResidual(*this, index);
}

double IECMPoint::evaluateEqualityViolation() const {
  return manifold()->equalityViolation(*this);
}

double IECMPoint::evaluateInequalityViolation() const {
  return manifold()->inequalityViolation(*this);
}

TangentSpaceMetric::shared_ptr IECMPoint::projectionMetric(
    const IERetractorParams& params,
    const std::optional<KeyVector>& priorKeys) const {
  const auto precisions =
      params.priorPrecisions(values_, priorKeys ? *priorKeys : values_.keys());
  return tangentSpace_->projectionMetric(values_, precisions,
                                         params.whitenAffineCoordinates);
}

IERetractor::shared_ptr IECMPoint::retractor(
    const VectorValues* metricSigmas) const {
  auto workspace = std::dynamic_pointer_cast<IERetractor>(
      retractionWorkspace(metricSigmas));
  if (!workspace)
    throw std::logic_error("Retraction policy has no IE boundary operations");
  return workspace;
}

}  // namespace gtdynamics
