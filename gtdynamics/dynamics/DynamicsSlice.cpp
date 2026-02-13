/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsSlice.cpp
 * @brief Dynamics factors for a single time slice.
 */

#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/factors/ContactKinematicsAccelFactor.h>
#include <gtdynamics/factors/TwistAccelFactor.h>
#include <gtdynamics/utils/utils.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;

NonlinearFactorGraph Dynamics::aFactors(
    const Slice& slice, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  NonlinearFactorGraph graph;
  for (auto&& link : robot.links()) {
    if (link->isFixed()) {
      graph.addPrior<gtsam::Vector6>(TwistAccelKey(link->id(), slice.k),
                                     gtsam::Z_6x1, p_.ba_cost_model);
    }
  }
  for (auto&& joint : robot.joints()) {
    graph.add(TwistAccelFactor(p_.a_cost_model, joint, slice.k));
  }

  if (contact_points) {
    for (auto&& cp : *contact_points) {
      ContactKinematicsAccelFactor contact_accel_factor(
          TwistAccelKey(cp.link->id(), slice.k), p_.ca_cost_model,
          gtsam::Pose3(gtsam::Rot3(), -cp.point));
      graph.add(contact_accel_factor);
    }
  }

  return graph;
}

}  // namespace gtdynamics
