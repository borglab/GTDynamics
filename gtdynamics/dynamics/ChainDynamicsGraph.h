/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ChainDynamicsGraph.h
 * @brief Builds a chain dynamics graph from a Robot object.
 * @author Dan Barladeanu
 */

#pragma once

#include <gtdynamics/dynamics/Chain.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/ContactDynamicsMomentFactor.h>
#include <gtdynamics/dynamics/WrenchFactor.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Vector3;

/**
 * ChainDynamicsGraph organizes a robot into chains (e.g., legs) and builds
 * nonlinear factor graphs for pose (q-level) and dynamics at a given time
 * step. It partitions the robot's joints into chain structures, composes
 * chain-level dynamics, and aggregates factors including optional contact
 * points and friction. The resulting graphs enable constrained optimization
 * with GTSAM for robots that can be modeled as a trunk with multiple chains.
 */
class ChainDynamicsGraph : public DynamicsGraph {
 private:
  std::vector<std::vector<JointSharedPtr>> chain_joints_;
  std::vector<Chain> composed_chains_;
  double trunk_mass_;

 public:
  /**
   * Constructor.
   * @param robot           the robot.
   * @param opt             settings for optimizer.
   * @param gravity         gravity in world frame.
   * @param planar_axis     axis of the plane, used only for planar robot.
   */
  ChainDynamicsGraph(const Robot &robot, const OptimizerSetting &opt,
                     const Vector3 &gravity = Vector3(0, 0, -9.8),
                     const std::optional<Vector3> &planar_axis = {})
      : DynamicsGraph(opt, gravity, planar_axis),
        chain_joints_(getChainJoints(robot)),
        composed_chains_(getComposedChains(chain_joints_)),
        trunk_mass_(robot.link("trunk")->mass()) {}

  // Destructor
  ~ChainDynamicsGraph() {}

  /// Return q-level nonlinear factor graph (pose related factors)
  NonlinearFactorGraph qFactors(
      const Robot &robot, const int t,
      const std::optional<PointOnLinks> &contact_points = {},
      double ground_plane_height = 0.0) const override;

  /**
   * Return nonlinear factor graph of all dynamics factors.
   * @param robot          the robot.
   * @param t              time step.
   * @param contact_points optional vector of contact points.
   * @param mu             optional coefficient of static friction.
   */
  NonlinearFactorGraph dynamicsFactors(
      const Robot &robot, const int t,
      const std::optional<PointOnLinks> &contact_points = {},
      const std::optional<double> &mu = {}) const override;

  /**
   * Return nonlinear factor graph of all dynamics factors.
   * @param robot          the robot.
   * @param t              time step.
   * @param contact_points optional vector of contact points.
   * @param mu             optional coefficient of static friction.
   */
  NonlinearFactorGraph dynamicsFactorGraph(
      const Robot &robot, const int t,
      const std::optional<PointOnLinks> &contact_points = {},
      const std::optional<double> &mu = {},
      double ground_plane_height = 0.0) const override;

  // Get a vector of legs, each leg is a vector of its joints
  static std::vector<std::vector<JointSharedPtr>> getChainJoints(
      const Robot &robot);

  // Get composed chains from chain joints
  static std::vector<Chain> getComposedChains(
      std::vector<std::vector<JointSharedPtr>> &chain_joints);
};

}  // namespace gtdynamics
