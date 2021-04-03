/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WrenchFactor.cpp
 * @brief Wrench balance factor, common between forward and inverse dynamics.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#include "gtdynamics/factors/WrenchFactor.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <vector>

using gtsam::Values;
using gtsam::Matrix;
using gtsam::Matrix63;
using gtsam::Matrix6;
using gtsam::Vector;
using gtsam::Vector6;
using gtsam::Pose3;
using gtsam::Point3;

namespace gtdynamics {

/// calculate coriolis term and jacobian w.r.t. joint coordinate twist
// TODO(gerry): replace with gtsam adjoint Jacobian
Vector6 coriolis(const Matrix6 &inertia, const Vector6 &twist,
                 gtsam::OptionalJacobian<-1, -1> H_twist = boost::none) {
  if (H_twist) {
    auto g1 = inertia(0, 0), g2 = inertia(1, 1), g3 = inertia(2, 2),
         m = inertia(3, 3);
    auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
         v2 = twist(4), v3 = twist(5);
    *H_twist = -(Matrix6() << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,  //
                 (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,               //
                 (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,               //
                 0, -m * v3, m * v2, 0, m * w3, -m * w2,                   //
                 m * v3, 0, -m * v1, -m * w3, 0, m * w1,                   //
                 -m * v2, m * v1, 0, m * w2, -m * w1, 0)
                    .finished();
  }
  return -Pose3::adjointMap(twist).transpose() * inertia * twist;
}

Vector WrenchFactor::unwhitenedError(
    const Values &x,
    boost::optional<std::vector<Matrix> &> H) const {
  if (!this->active(x)) {
    return Vector::Zero(this->dim());
  }

  // `keys_` order: twist, twistAccel, pose, *wrenches
  const Vector6 twist = x.at<Vector6>(keys_.at(0));
  const Vector6 twistAccel = x.at<Vector6>(keys_.at(1));
  const Pose3 pose = x.at<Pose3>(keys_.at(2));
  Vector6 wrenchSum = gtsam::Z_6x1;
  for (auto key = keys_.cbegin() + 3; key != keys_.cend(); ++key) {
    wrenchSum += x.at<Vector6>(*key);
  }

  // transform gravity from base frame to link COM frame,
  // to use unrotate function, have to convert gravity vector to a point
  Vector6 gravity_wrench;
  Matrix H_rotation, H_unrotate;
  Matrix63 intermediateMatrix;
  if (gravity_) {
    auto gravity =
        pose.rotation(H_rotation).unrotate(*gravity_, H_unrotate);
    intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
    gravity_wrench = inertia_ * intermediateMatrix * gravity;
    if (H) (*H)[2] = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
  } else {
    gravity_wrench = gtsam::Z_6x1;
    if (H) (*H)[2] = gtsam::Z_6x6;
  }

  // Equation 8.48 (F = ma)
  Vector6 error = (inertia_ * twistAccel) +
                  coriolis(inertia_, twist, H ? &(*H)[0] : 0)  //
                  - wrenchSum - gravity_wrench;

  if (H) {
    (*H)[1] = inertia_;
    std::fill(H->begin()+3, H->end(), -gtsam::I_6x6);
  }

  return error;
}

}  // namespace gtdynamics
