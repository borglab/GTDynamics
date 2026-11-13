/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CMPoint.cpp
 * @brief Shared point state, recovery, and tangent coordinates.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/constrained/CMPoint.h>
#include <gtdynamics/constrained/CMRetractor.h>

#include <iostream>

namespace gtdynamics {
using namespace gtsam;

CMPoint& CMPoint::operator=(const CMPoint& other) {
  if (this == &other) return *this;
  validateAssignment(other);
  if (manifold_ &&
      (manifold_ != other.manifold_ || bindings_ != other.bindings_))
    throw std::invalid_argument(
        "CMPoint assignment requires the same type and bindings");

  // Default values may initialize once; existing points keep their identity.
  if (!manifold_) {
    bindings_ = other.bindings_;
    manifold_ = other.manifold_;
  }
  values_ = other.values_;
  tangentSpace_ = other.tangentSpace_;
  retractionPreparation_ = other.retractionPreparation_;
  return *this;
}

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

std::shared_ptr<CMRetractor> CMPoint::retractionWorkspace(
    const VectorValues* metricSigmas) const {
  if (!retractionPreparation_)
    throw std::logic_error("Native point owns its chart retraction");

  auto workspace = retractionPreparation_->cloneWorkspace();
  if (metricSigmas) {
    // Copy only this point's entries from a possibly trajectory-wide metric.
    VectorValues local;
    for (Key key : values_.keys())
      if (metricSigmas->exists(key)) local.insert(key, metricSigmas->at(key));
    workspace->setMetricSigmas(local);
  }

  return workspace;
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
  return manifold_->recover(*this, key, H);
}

const Value& CMPoint::recoverSlot(
    Key slot, OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H) const {
  if (!manifold_)
    throw std::invalid_argument("Cannot recover a slot from an unbound CMPoint");
  return recover(bindings_.at(slot), H);
}

}  // namespace gtdynamics
