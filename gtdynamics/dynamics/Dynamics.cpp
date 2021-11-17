/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Dynamics.cpp
 * @brief Wrench calculations for configurations in motion.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

using gtsam::Matrix6;
using gtsam::Vector6;

Vector6 Coriolis(const Matrix6 &inertia, const Vector6 &twist,
                 gtsam::OptionalJacobian<6, 6> H_twist) {
  if (H_twist) {
    // TODO(gerry): replace with gtsam adjoint Jacobian
    auto g1 = inertia(0, 0), g2 = inertia(1, 1), g3 = inertia(2, 2),
         m = inertia(3, 3);
    auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
         v2 = twist(4), v3 = twist(5);
    *H_twist = (Matrix6() << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,  //
                (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,               //
                (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,               //
                0, -m * v3, m * v2, 0, m * w3, -m * w2,                   //
                m * v3, 0, -m * v1, -m * w3, 0, m * w1,                   //
                -m * v2, m * v1, 0, m * w2, -m * w1, 0)
                   .finished();
  }
  auto result = gtsam::Pose3::adjointTranspose(twist, inertia * twist);
  return result;
}

Vector6 MatVecMult(const Matrix6 &inertia, const Vector6 &twist,
                 gtsam::OptionalJacobian<6, 6> H_twist) {
  if (H_twist) {
      *H_twist = inertia;
      }
  auto result = inertia * twist;
  return result;
}

gtsam::Vector3 MatVecMult36(const gtsam::Matrix36 &mat, const Vector6 &vec,
                 gtsam::OptionalJacobian<3, 6> H_vec) {
  if (H_vec) {
      *H_vec = mat;
      }
  auto result = mat * vec;
  return result;
}

}  // namespace gtdynamics
