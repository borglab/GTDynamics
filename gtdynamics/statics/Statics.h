/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Statics.h
 * @brief Wrench calculations for configurations at rest.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/// Calculate gravity wrench
gtsam::Vector6 GravityWrench(
    const gtsam::Vector3 &gravity, const gtsam::Matrix6 &inertia,
    const gtsam::Pose3 &wTcom,
    gtsam::OptionalJacobian<6, 6> H_pose = boost::none) {
  // transform gravity from base frame to link COM frame,
  gtsam::Matrix36 H_rotation;
  gtsam::Matrix33 H_unrotate;
  gtsam::Matrix63 intermediateMatrix;
  const gtsam::Rot3 wRcom = wTcom.rotation(H_pose ? &H_rotation : nullptr);
  auto gravity_com = wRcom.unrotate(gravity, H_pose ? &H_unrotate : nullptr);

  intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
  if (H_pose) {
    *H_pose = inertia * intermediateMatrix * H_unrotate * H_rotation;
  }
  return inertia * intermediateMatrix * gravity_com;
}

}  // namespace gtdynamics
