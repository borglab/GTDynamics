/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.cpp
 * @brief Absract representation of a robot joint.
 */

#include "gtdynamics/universal_robot/Link.h"

#include <gtsam/slam/expressions.h>

#include <iostream>

#include "gtdynamics/dynamics/Dynamics.h"
#include "gtdynamics/statics/Statics.h"

namespace gtdynamics {

gtsam::Vector6_ Link::wrenchConstraint(
    const std::vector<DynamicsSymbol>& wrench_keys, uint64_t t,
    const boost::optional<gtsam::Vector3>& gravity) const {
  // Collect wrenches to implement L&P Equation 8.48 (F = ma)
  std::vector<gtsam::Vector6_> wrenches;

  // Coriolis forces.
  gtsam::Vector6_ twist(internal::TwistKey(id(), t));
  gtsam::Vector6_ wrench_coriolis(
      std::bind(Coriolis, inertiaMatrix(), std::placeholders::_1,
                std::placeholders::_2),
      twist);
  wrenches.push_back(wrench_coriolis);

  // Change in generalized momentum.
  const gtsam::Matrix6 neg_inertia = -inertiaMatrix();
  gtsam::Vector6_ twistAccel(internal::TwistAccelKey(id(), t));
  gtsam::Vector6_ wrench_momentum(
      std::bind(matrixMult, neg_inertia, std::placeholders::_1,
                std::placeholders::_2),
      twistAccel);
  wrenches.push_back(wrench_momentum);


  // External wrenches.
  for (const auto& key : wrench_keys) {
    wrenches.push_back(gtsam::Vector6_(key));
  }

  // Gravity wrench.
  if (gravity) {
    gtsam::Pose3_ pose(internal::PoseKey(id(), t));
    gtsam::Vector6_ wrench_gravity(
        std::bind(GravityWrench, *gravity, mass_, std::placeholders::_1,
                  std::placeholders::_2),
        pose);
    wrenches.push_back(wrench_gravity);
  }

  // Calculate resultant wrench.
  gtsam::Vector6_ error(gtsam::Z_6x1);
  for (const auto& wrench : wrenches) {
    error += wrench;
  }

  return error;
}

}  // namespace gtdynamics