/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file ConstraintManifold.h
 * @brief Common runtime contract for constrained ambient variables.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <memory>

namespace gtdynamics {
class CMPoint;
class CMTangentSpace;

// Runtime types describe charts independently of factor-graph storage.
class ConstraintManifold {
 public:
  using shared_ptr = std::shared_ptr<const ConstraintManifold>;
  using Bindings = std::map<gtsam::Key, gtsam::Key>;
  virtual ~ConstraintManifold() = default;
  virtual size_t dim() const = 0;
  virtual std::map<gtsam::Key, size_t> dimensions() const = 0;
  virtual bool affineEqualities() const = 0;
  virtual shared_ptr conditionBase(const gtsam::Values& fixed) const = 0;
  virtual std::shared_ptr<const CMTangentSpace> linearize(
      const CMPoint& point) const = 0;
  virtual const gtsam::Value& recover(
      const CMPoint& point, gtsam::Key key,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) const;
  virtual gtsam::Values retractValues(const CMPoint& point,
                                      const gtsam::Vector& delta) const = 0;
};
}  // namespace gtdynamics
