/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtdynamics/universal_robot/Link.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/** A material frame fixed to a link. comTcontact maps contact coordinates into
 * the link COM frame. Contact +Z is the outward ground normal (up for a level
 * sole); +X is forward and +Y left. No extra state variable is introduced.
 */
struct PoseOnLink {
  LinkSharedPtr link;
  gtsam::Pose3 comTcontact;
  bool isGround = true;

  PoseOnLink() = default;
  PoseOnLink(const LinkSharedPtr& link, const gtsam::Pose3& comTcontact,
             bool isGround = true)
      : link(link), comTcontact(comTcontact), isGround(isGround) {}

  /// World contact pose from the link COM pose at source slice k.
  gtsam::Pose3 predict(const gtsam::Values& values, size_t k = 0) const {
    return values.at<gtsam::Pose3>(PoseKey(link->id(), k)) * comTcontact;
  }
  /// Compare link identity, fixed transform, and ground role.
  bool equals(const PoseOnLink& other, double tol = 1e-9) const {
    return link == other.link && isGround == other.isGround &&
           comTcontact.equals(other.comTcontact, tol);
  }
};
}  // namespace gtdynamics
