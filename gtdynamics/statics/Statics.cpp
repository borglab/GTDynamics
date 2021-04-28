/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Statics.cpp
 * @brief Wrench calculations for configurations at rest.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

gtsam::Vector6 GravityWrench(
    const gtsam::Vector3 &gravity, double mass, const gtsam::Pose3 &wTcom,
    gtsam::OptionalJacobian<6, 6> H_wTcom = boost::none) {
  // Transform gravity from base frame to link COM frame.
  gtsam::Matrix33 H_unrotate;
  const gtsam::Rot3 wRcom = wTcom.rotation();
  auto gravity_com = wRcom.unrotate(gravity, H_wTcom ? &H_unrotate : nullptr);

  // Compose wrench.
  gtsam::Vector6 wrench;
  wrench << 0, 0, 0, mass * gravity_com;

  // Calculate derivatives if asked.
  if (H_wTcom) {
    H_wTcom->setZero();
    H_wTcom->bottomLeftCorner<3, 3>() = mass * H_unrotate;
  }

  return wrench;
}

}  // namespace gtdynamics
