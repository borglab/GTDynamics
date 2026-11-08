/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CMPoint.h
 * @brief Shared state and recovery for concrete constrained-manifold points.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/constrained/CMTangentSpace.h>
#include <gtdynamics/constrained/ConstraintManifold.h>

namespace gtdynamics {
class IERetractor;
class IEConstraintManifold;
class EConstraintManifold;

// The common base is used by reference; GTSAM stores concrete point values.
class CMPoint {
 public:
  using Bindings = ConstraintManifold::Bindings;
  virtual ~CMPoint() = default;
  const gtsam::Values& values() const { return values_; }
  const ConstraintManifold::shared_ptr& manifold() const { return manifold_; }
  const Bindings& bindings() const { return bindings_; }
  size_t dim() const { return manifold_ ? manifold_->dim() : 0; }
  bool hasAffineEqualities() const { return manifold_->affineEqualities(); }
  const std::shared_ptr<const CMTangentSpace>& tangentSpace() const {
    return tangentSpace_;
  }
  const std::shared_ptr<const TangentSpaceBasis>& eBasis() const;
  gtsam::VectorValues toAmbient(const gtsam::Vector& delta) const;
  gtsam::Vector toCoordinates(const gtsam::VectorValues& delta) const;
  const gtsam::Value& recover(
      gtsam::Key key,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) const;
  bool equals(const CMPoint& other, double tolerance = 1e-8) const;
  void print(const std::string& prefix = "") const;

 protected:
  CMPoint() = default;
  CMPoint(const CMPoint&) = default;
  CMPoint& operator=(const CMPoint& other);
  // Concrete point categories can enforce stronger type guarantees.
  virtual void validateAssignment(const CMPoint&) const {}
  friend class IEConstraintManifold;
  friend class EConstraintManifold;
  ConstraintManifold::shared_ptr manifold_;
  Bindings bindings_;
  gtsam::Values values_;
  std::shared_ptr<const CMTangentSpace> tangentSpace_;
  std::shared_ptr<const IERetractor> retractionPreparation_;
};
}  // namespace gtdynamics
