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
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

#include "gtdynamics/dynamics/Dynamics.h"
#include "gtdynamics/statics/Statics.h"

using gtsam::Matrix;
using gtsam::Matrix6;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;

namespace gtdynamics {

WrenchFactor::WrenchFactor(
    gtsam::Key twist_key, gtsam::Key twistAccel_key,
    const std::vector<DynamicsSymbol> &wrench_keys, gtsam::Key pose_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const Matrix6 &inertia, const boost::optional<gtsam::Vector3> &gravity)
    : Base(cost_model), inertia_(inertia), gravity_(gravity) {
  keys_.reserve(wrench_keys.size() + 3);
  keys_.push_back(twist_key);
  keys_.push_back(twistAccel_key);
  keys_.insert(keys_.end(), wrench_keys.cbegin(), wrench_keys.cend());
  keys_.push_back(pose_key);
}

Vector WrenchFactor::unwhitenedError(
    const Values &x, boost::optional<std::vector<Matrix> &> H) const {
  if (!this->active(x)) {
    return Vector::Zero(this->dim());
  }

  // Collect wrenches to implement L&P Equation 8.48 (F = ma)
  std::vector<Vector6> wrenches;

  // Coriolis forces.
  const Vector6 twist = x.at<Vector6>(keys_.at(0));
  Matrix6 H_twist;
  wrenches.push_back(Coriolis(inertia_, twist, H ? &H_twist : 0));

  // Change in generalized momentum.
  const Vector6 twistAccel = x.at<Vector6>(keys_.at(1));
  wrenches.push_back(-inertia_ * twistAccel);

  // External wrenches.
  for (auto key = keys_.cbegin() + 2; key != keys_.cend() - 1; ++key) {
    wrenches.push_back(x.at<Vector6>(*key));
  }

  // Calculate resultant wrench, fills up H with identity matrices if asked,
  // except the last H contains the pose derivative or zero if no gravity.
  const Vector6 error = TotalExternalWrench(
      wrenches, inertia_(3, 3), x.at<Pose3>(keys_.back()), gravity_, H);

  // If asked, update Jacobians not yet calculated by TotalExternalWrench.
  if (H) {
    (*H)[0] = H_twist;    // Coriolis depends in twist (key 0)
    (*H)[1] = -inertia_;  // Derivative with respect to twist acceleration
  }

  return error;
}

}  // namespace gtdynamics
