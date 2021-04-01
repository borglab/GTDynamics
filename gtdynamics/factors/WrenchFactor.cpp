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

#include <gtdynamics/factors/WrenchFactor.h>
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/DynamicsSymbol.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/assign/list_of.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/base_object.hpp>
#include <string>
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

Matrix6 WrenchFactor::twistJacobian_(const gtsam::Vector6 &twist) const {
  auto g1 = inertia_(0, 0), g2 = inertia_(1, 1), g3 = inertia_(2, 2),
        m = inertia_(3, 3);
  auto w1 = twist(0), w2 = twist(1), w3 = twist(2), v1 = twist(3),
        v2 = twist(4), v3 = twist(5);
  Matrix6 H_twist;
  H_twist << 0, (g2 - g3) * w3, (g2 - g3) * w2, 0, 0, 0,  //
      (g3 - g1) * w3, 0, (g3 - g1) * w1, 0, 0, 0,         //
      (g1 - g2) * w2, (g1 - g2) * w1, 0, 0, 0, 0,         //
      0, -m * v3, m * v2, 0, m * w3, -m * w2,             //
      m * v3, 0, -m * v1, -m * w3, 0, m * w1,             //
      -m * v2, m * v1, 0, m * w2, -m * w1, 0;
  return H_twist;
}

Vector WrenchFactor::unwhitenedError(
    const Values &x,
    boost::optional<std::vector<Matrix> &> H = boost::none) const {
  if (!this->active(x)) {
    return Vector::Zero(this->dim());
  }

  // `keys_` order: twist, twistAccel, pose, *wrenches
  const Vector6 twist = x.at<Vector6>(keys_.at(0));
  const Vector6 twistAccel = x.at<Vector6>(keys_.at(1));
  const Pose3 pose = x.at<Pose3>(keys_.at(2));
  Vector6 wrenchSum = gtsam::Z_6x1;
  for (auto key = keys_.cbegin() + 3; key != keys_.cend(); ++key) {
    wrenchSum += x.at<Vector6>(key);
  }

  // transform gravity from base frame to link COM frame,
  // to use unrotate function, have to convert gravity vector to a point
  Point3 gravity_point(gravity_[0], gravity_[1], gravity_[2]);
  Matrix H_rotation, H_unrotate;
  auto gravity =
      pose.rotation(H_rotation).unrotate(gravity_point, H_unrotate);
  Matrix63 intermediateMatrix;
  intermediateMatrix << gtsam::Z_3x3, gtsam::I_3x3;
  auto gravity_wrench = inertia_ * intermediateMatrix * gravity;

  // Equation 8.48 (F = ma)
  Vector6 error =
      (inertia_ * twistAccel) - wrenchSum - gravity_wrench -
      (Pose3::adjointMap(twist).transpose() * inertia_ * twist);

  if (H) {
    (*H)[0] = -twistJacobian_(twist);
    (*H)[1] = inertia_;
    (*H)[2] = -inertia_ * intermediateMatrix * H_unrotate * H_rotation;
    std::fill(H->begin()+3, H->end(), -gtsam::I_6x6);
  }

  return error;
}

}  // namespace gtdynamics
