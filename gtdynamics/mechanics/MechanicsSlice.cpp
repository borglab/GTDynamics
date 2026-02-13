/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MechanicsSlice.cpp
 * @brief Mechanics factors in a single time slice.
 */

#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchPlanarFactor.h>
#include <gtdynamics/mechanics/Mechanics.h>

namespace gtdynamics {

gtsam::NonlinearFactorGraph Mechanics::wrenchEquivalenceFactors(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(WrenchEquivalenceFactor(p_.f_cost_model, joint, slice.k));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Mechanics::torqueFactors(const Slice& slice,
                                                     const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(TorqueFactor(p_.t_cost_model, joint, slice.k));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Mechanics::wrenchPlanarFactors(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  if (p_.planar_axis) {
    for (auto&& joint : robot.joints()) {
      graph.add(WrenchPlanarFactor(p_.planar_cost_model, *p_.planar_axis, joint,
                                   slice.k));
    }
  }
  return graph;
}

}  // namespace gtdynamics
