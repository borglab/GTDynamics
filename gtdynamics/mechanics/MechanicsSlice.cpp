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

#include <gtdynamics/mechanics/TorqueFactor.h>
#include <gtdynamics/mechanics/WrenchEquivalenceFactor.h>
#include <gtdynamics/mechanics/WrenchPlanarFactor.h>
#include <gtdynamics/mechanics/Mechanics.h>

namespace gtdynamics {

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchEquivalenceFactors<Slice>(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(WrenchEquivalenceFactor(p_.f_cost_model, joint, slice.k));
  }
  return graph;
}

template <>
gtsam::NonlinearFactorGraph Mechanics::torqueFactors<Slice>(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(TorqueFactor(p_.t_cost_model, joint, slice.k));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Mechanics::torqueFactors(
    const Slice& slice, const Robot& robot, const gtsam::Vector& armature) const {
  if (!armature.allFinite() || (armature.array() < 0).any())
    throw std::invalid_argument("Armature must be finite and nonnegative");
  gtsam::NonlinearFactorGraph graph;
  for (const auto& joint : robot.joints()) {
    if (armature.size() && (joint->id() < 0 || joint->id() >= armature.size()))
      throw std::invalid_argument("Armature must be indexed by joint id");
    graph.add(TorqueFactor(p_.t_cost_model, joint, slice.k,
                          armature.size() ? armature(joint->id()) : 0.0));
  }
  return graph;
}

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchPlanarFactors<Slice>(
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
