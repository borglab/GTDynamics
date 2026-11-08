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

#include <iostream>

namespace gtdynamics {
using namespace gtsam;

CMPoint& CMPoint::operator=(const CMPoint& other) {
  if (this == &other) return *this;
  validateAssignment(other);
  manifold_ = other.manifold_;
  bindings_ = other.bindings_;
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

}  // namespace gtdynamics
