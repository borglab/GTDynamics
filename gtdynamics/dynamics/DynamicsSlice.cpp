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
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactKinematicsAccelFactor.h>
#include <gtdynamics/factors/TwistAccelFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
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

NonlinearFactorGraph Dynamics::graph(
    const Slice& slice, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const std::optional<double>& mu) const {
  NonlinearFactorGraph graph;
  const double friction_coefficient = mu ? *mu : 1.0;
  const gtsam::Vector3 gravity =
      p_.gravity.value_or(gtsam::Vector3(0.0, 0.0, -9.81));

  for (auto&& link : robot.links()) {
    const int i = link->id();
    if (link->isFixed()) {
      continue;
    }

    const auto& connected_joints = link->joints();
    std::vector<gtsam::Key> wrench_keys;

    // Add wrench keys for joints.
    for (auto&& joint : connected_joints) {
      wrench_keys.push_back(WrenchKey(i, joint->id(), slice.k));
    }

    // Add wrench keys for contact points.
    if (contact_points) {
      for (auto&& cp : *contact_points) {
        if (cp.link->id() != i) {
          continue;
        }
        // TODO(frank): allow multiple contact points on one link, id = 0,1,..
        const auto wrench_key = ContactWrenchKey(i, 0, slice.k);
        wrench_keys.push_back(wrench_key);

        graph.emplace_shared<ContactDynamicsFrictionConeFactor>(
            PoseKey(i, slice.k), wrench_key, p_.cfriction_cost_model,
            friction_coefficient, gravity);

        graph.emplace_shared<ContactDynamicsMomentFactor>(
            wrench_key, p_.cm_cost_model, gtsam::Pose3(gtsam::Rot3(), -cp.point));
      }
    }

    graph.add(WrenchFactor(p_.fa_cost_model, link, wrench_keys, slice.k,
                           p_.gravity));
  }

  return graph;
}

}  // namespace gtdynamics
