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
using gtsam::Matrix63;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;

namespace gtdynamics {

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

  // Gravity wrench.
  Matrix6 H_wTcom;
  if (gravity_) {
    const Pose3 wTcom = x.at<Pose3>(keys_.at(2));
    wrenches.push_back(GravityWrench(*gravity_, inertia_(3, 3), wTcom,
                                     H ? &H_wTcom : nullptr));
  } else {
    wrenches.push_back(gtsam::Z_6x1);
  }

  // External wrenches.
  for (auto key = keys_.cbegin() + 3; key != keys_.cend(); ++key) {
    wrenches.push_back(x.at<Vector6>(*key));
  }

  // Calculate resultant wrench, fills up H with identity matrices if asked.
  Vector6 error = ResultantWrench(wrenches, H);
  if (H) {
    (*H)[0] = H_twist;
    (*H)[1] = -inertia_;
    if (gravity_) {
      (*H)[2] = H_wTcom;
    } else {
      (*H)[2] = gtsam::Z_6x6;
    }
  }

  return error;
}

}  // namespace gtdynamics
