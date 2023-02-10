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
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtdynamics {

using gtsam::Vector6;

Vector6 GravityWrench(const gtsam::Vector3 &gravity, double mass,
                      const gtsam::Pose3 &wTcom,
                      gtsam::OptionalJacobian<6, 6> H_wTcom = {}) {
  // Transform gravity from base frame to link COM frame.
  gtsam::Matrix33 H_unrotate;
  const gtsam::Rot3 wRcom = wTcom.rotation();
  auto gravity_com = wRcom.unrotate(gravity, H_wTcom ? &H_unrotate : nullptr);

  // Compose wrench.
  Vector6 wrench;
  wrench << 0, 0, 0, mass * gravity_com;

  // Calculate derivatives if asked.
  if (H_wTcom) {
    H_wTcom->setZero();
    H_wTcom->bottomLeftCorner<3, 3>() = mass * H_unrotate;
  }

  return wrench;
}

Vector6 ResultantWrench(const std::vector<gtsam::Vector6> &wrenches,
                        gtsam::OptionalMatrixVecType H) {
  Vector6 sum = gtsam::Z_6x1;
  const size_t n = wrenches.size();
  for (size_t i = 0; i < n; i++) {
    sum += wrenches[i];
  }
  if (H) {
    std::fill(H->begin(), H->begin() + n, gtsam::I_6x6);
  }
  return sum;
}

Vector6 ResultantWrench(const std::vector<Vector6> &wrenches, double mass,
                        const gtsam::Pose3 &wTcom,
                        std::optional<gtsam::Vector3> gravity,
                        gtsam::OptionalMatrixVecType H) {
  // Calculate resultant wrench, fills up H with identity matrices if asked.
  const Vector6 external_wrench = ResultantWrench(wrenches, H);

  // Potentiall add gravity wrench.
  if (gravity) {
    gtsam::Matrix6 H_wTcom;
    auto gravity_wrench =
        GravityWrench(*gravity, mass, wTcom, H ? &H_wTcom : nullptr);
    if (H) {
      H->back() = H_wTcom;
    }
    return external_wrench + gravity_wrench;
  } else {
    if (H) {
      H->back() = gtsam::Z_6x6;
    }
    return external_wrench;
  }
}

}  // namespace gtdynamics
