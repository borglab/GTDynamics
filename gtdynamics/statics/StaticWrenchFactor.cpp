/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticWrenchFactor.cpp
 * @brief Wrench balance factor for a stationary link.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#include <gtdynamics/statics/StaticWrenchFactor.h>
#include <gtdynamics/statics/Statics.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <vector>

using gtsam::Matrix;
using gtsam::Pose3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;

namespace gtdynamics {

StaticWrenchFactor::StaticWrenchFactor(
    const std::vector<gtsam::Key> &wrench_keys, gtsam::Key pose_key,
    const gtsam::noiseModel::Base::shared_ptr &cost_model, double mass,
    const boost::optional<gtsam::Vector3> &gravity)
    : Base(cost_model, wrench_keys), mass_(mass), gravity_(gravity) {
  keys_.push_back(pose_key);
}

Vector StaticWrenchFactor::unwhitenedError(
    const Values &x, boost::optional<std::vector<Matrix> &> H) const {
  if (!this->active(x)) {
    return Vector::Zero(this->dim());
  }

  // Collect external wrenches.
  std::vector<Vector6> external_wrenches;
  for (auto key = keys_.cbegin(); key != keys_.cend() - 1; ++key) {
    external_wrenches.push_back(x.at<Vector6>(*key));
  }

  // H->back() will contain Jacobian with respect to pose.
  return ResultantWrench(external_wrenches, mass_, x.at<Pose3>(keys_.back()),
                         gravity_, H);
}

}  // namespace gtdynamics
