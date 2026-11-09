/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CMPoint.cpp
 * @brief Values and GTSAM chart operations for runtime manifold types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/CMPoint.h>

#include <iostream>

namespace gtdynamics {
const std::shared_ptr<const TangentSpaceBasis>& CMPoint::eBasis() const {
  if (!tangentSpace_)
    throw std::invalid_argument("Unbound CMPoint has no tangent space");
  return tangentSpace_->basis();
}

VectorValues CMPoint::toAmbient(const Vector& delta) const {
  if (delta.size() != dim())
    throw std::invalid_argument("Incorrect CMPoint tangent dimension");
  return dim() ? eBasis()->computeTangentVector(delta) : values_.zeroVectors();
}

Vector CMPoint::toCoordinates(const VectorValues& delta) const {
  return dim() ? eBasis()->computeXi(delta) : Vector(0);
}

CMPoint CMPoint::retract(const Vector& delta) const {
  return retract(delta, {});
}

CMPoint CMPoint::retract(const Vector& delta,
                         const std::optional<IndexSet>& blocking,
                         IERetractionInfo* info,
                         const VectorValues* metricSigmas) const {
  if (!manifold_)
    throw std::invalid_argument("Cannot retract an unbound CMPoint");
  if (delta.size() != dim())
    throw std::invalid_argument("Incorrect CMPoint tangent dimension");
  return manifold_->retract(*this, delta, blocking, info, metricSigmas);
}

CMPoint CMPoint::retract(const VectorValues& delta,
                         const std::optional<IndexSet>& blocking,
                         IERetractionInfo* info,
                         const VectorValues* metricSigmas) const {
  if (!manifold_)
    throw std::invalid_argument("Cannot retract an unbound CMPoint");
  return manifold_->retractAmbient(*this, delta, blocking, info, metricSigmas);
}

Vector CMPoint::localCoordinates(const CMPoint& other) const {
  if (!manifold_ || manifold_ != other.manifold_ ||
      bindings_ != other.bindings_)
    throw std::invalid_argument(
        "CMPoint local coordinates require the same type and bindings");
  return manifold_->localCoordinates(*this, other);
}

bool CMPoint::equals(const CMPoint& other, double tolerance) const {
  return manifold_ == other.manifold_ && bindings_ == other.bindings_ &&
         values_.equals(other.values_, tolerance);
}

void CMPoint::print(const std::string& prefix) const {
  std::cout << prefix << "CMPoint (dimension " << dim() << ")\n";
  values_.print();
}

const Value& CMPoint::recover(
    Key key, OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H) const {
  // The recovery Jacobian maps an ordinary point tangent to this ambient value.
  if (H)
    *H = dim() ? eBasis()->recoverJacobian(key)
               : Matrix::Zero(values_.at(key).dim(), 0);
  return values_.at(key);
}

std::pair<IndexSet, Vector> CMPoint::projectTangentCone(
    const Vector& delta) const {
  return tangentSpace_->project(delta);
}

std::pair<IndexSet, VectorValues> CMPoint::projectTangentCone(
    const VectorValues& delta) const {
  auto [active, reduced] = projectTangentCone(toCoordinates(delta));
  return {active, toAmbient(reduced)};
}

IndexSet CMPoint::blockingIndices(const VectorValues& delta) const {
  return tangentSpace_->blockingIndices(delta);
}

CMPoint CMPoint::moveToBoundary(const IndexSet& active, IERetractionInfo* info,
                                const VectorValues* metricSigmas) const {
  return manifold_->moveToBoundary(*this, active, info, metricSigmas);
}

Vector CMPoint::inequalityResidual(size_t index) const {
  return manifold_->inequalityResidual(*this, index);
}

double CMPoint::evaluateEqualityViolation() const {
  return manifold_->equalityViolation(*this);
}

double CMPoint::evaluateInequalityViolation() const {
  return manifold_->inequalityViolation(*this);
}

TangentSpaceMetric::shared_ptr CMPoint::projectionMetric(
    const IERetractorParams& params,
    const std::optional<KeyVector>& priorKeys) const {
  const auto precisions =
      params.priorPrecisions(values_, priorKeys ? *priorKeys : values_.keys());
  return tangentSpace_->projectionMetric(values_, precisions,
                                         params.whitenAffineCoordinates);
}

IERetractor::shared_ptr CMPoint::retractor(
    const VectorValues* metricSigmas) const {
  if (!retractionPreparation_)
    throw std::logic_error("Native point owns its chart retraction");

  auto workspace = retractionPreparation_->clone();
  if (metricSigmas) {
    // Copy only this point's entries from a possibly trajectory-wide metric.
    VectorValues local;
    for (Key key : values_.keys())
      if (metricSigmas->exists(key)) local.insert(key, metricSigmas->at(key));
    workspace->setMetricSigmas(local);
  }

  return workspace;
}

}  // namespace gtdynamics
