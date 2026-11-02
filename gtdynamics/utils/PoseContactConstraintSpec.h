/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtdynamics/utils/ConstraintSpec.h>

#include <iostream>

namespace gtdynamics {

/** Active material frames for a kinematic phase. Point-contact builders see an
 * empty point collection; use the explicit pose-contact builders with this
 * spec.
 */
class PoseContactConstraintSpec : public ConstraintSpec {
  std::vector<PoseOnLink> contacts_;
  PointOnLinks points_;

 public:
  PoseContactConstraintSpec() = default;
  explicit PoseContactConstraintSpec(const std::vector<PoseOnLink>& contacts)
      : contacts_(contacts) {}
  const std::vector<PoseOnLink>& contactPoses() const override {
    return contacts_;
  }
  const PointOnLinks& contactPoints() const override { return points_; }
  void print(const std::string& s = "") const override {
    std::cout << s << "PoseContactConstraintSpec: " << contacts_.size()
              << " frames\n";
  }
};
}  // namespace gtdynamics
