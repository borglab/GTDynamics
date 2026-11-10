/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file ECMPoint.cpp
 * @brief Typed chart wrappers and collection operations for equality points.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#include <gtdynamics/cmopt/ECMPoint.h>

namespace gtdynamics {
void ECMPoint::validateAssignment(const CMPoint& other) const {
  // A base reference must preserve the equality-only runtime type.
  if (other.manifold() &&
      !std::dynamic_pointer_cast<const EConstraintManifold>(other.manifold()))
    throw std::invalid_argument("An equality point requires an equality runtime type");
}
ECMPoint ECMPoint::fromIEPoint(const IECMPoint& point) {
  if (!std::dynamic_pointer_cast<const EConstraintManifold>(point.manifold()))
    throw std::invalid_argument(
        "An equality point requires an equality runtime type");
  return ECMPoint(point);
}
ECMPoint ECMPoint::retract(const Vector& delta) const {
  return ECMPoint(IECMPoint::retract(delta));
}
ECMPoint ECMPoint::retract(const Vector& delta,
                           const std::optional<IndexSet>& blocking,
                           IERetractionInfo* info,
                           const VectorValues* metricSigmas) const {
  return ECMPoint(IECMPoint::retract(delta, blocking, info, metricSigmas));
}
ECMPoint ECMPoint::retract(const VectorValues& delta,
                           const std::optional<IndexSet>& blocking,
                           IERetractionInfo* info,
                           const VectorValues* metricSigmas) const {
  return ECMPoint(IECMPoint::retract(delta, blocking, info, metricSigmas));
}
Values ambientValues(const ECMPoints& points) {
  Values result;
  for (const auto& [key, point] : points) result.insert(point.values());
  return result;
}
KeyVector pointKeys(const ECMPoints& points) {
  KeyVector result;
  for (const auto& [key, point] : points) result.push_back(key);
  return result;
}
IECMPoints widenPoints(const ECMPoints& points) {
  IECMPoints result;
  for (const auto& [key, point] : points) result.emplace(key, point);
  return result;
}
ECMPoints extractECMPoints(const Values& values) {
  ECMPoints result;
  for (Key key : values.keys()) {
    if (const auto* value =
            dynamic_cast<const gtsam::GenericValue<ECMPoint>*>(&values.at(key)))
      result.emplace(key, value->value());
  }
  return result;
}
VectorValues computeTangentVector(const ECMPoints& points,
                                  const VectorValues& delta) {
  VectorValues result;
  for (const auto& [key, point] : points)
    result.insert(point.toAmbient(delta.at(key)));
  return result;
}
ECMPoints retractPoints(const ECMPoints& points, const VectorValues& delta) {
  ECMPoints result;
  for (const auto& [key, point] : points)
    result.emplace(key, point.retract(delta.at(key)));
  return result;
}
}  // namespace gtdynamics
