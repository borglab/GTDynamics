/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file LeanDynamicsGraph.h
 * @brief Builds a Lean dynamics graph from a Robot object.
 * @author Dan Barladeanu
 */

#pragma once


#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/dynamics/Chain.h"
#include <gtdynamics/optimizer/EqualityConstraint.h>

namespace gtdynamics {


class LeanDynamicsGraph : public DynamicsGraph {

  private:
  std::vector<std::vector<JointSharedPtr>> chain_joints_;
  std::vector<Chain> composed_chains_;
  gtsam::Vector3 tolerance_;

  public:

    /**
     * Constructor
     * @param  opt          settings for optimizer
     * @param  gravity      gravity in world frame
     * @param  planar_axis  axis of the plane, used only for planar robot
     */
    LeanDynamicsGraph(
        const OptimizerSetting &opt,
        const boost::optional<gtsam::Vector3> &gravity = boost::none,
        const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
        : DynamicsGraph(opt, gravity, planar_axis) {}

    /**
     * Constructor
     * @param  opt          settings for optimizer
     * @param  gravity      gravity in world frame
     * @param  planar_axis  axis of the plane, used only for planar robot
     */
    LeanDynamicsGraph(
        const OptimizerSetting &opt,
        const std::vector<Chain> &composed_chains,
        const std::vector<std::vector<JointSharedPtr>> &chain_joints,
        const gtsam::Vector3 &tolerance,
        const boost::optional<gtsam::Vector3> &gravity = boost::none,
        const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
        : DynamicsGraph(opt, gravity, planar_axis), chain_joints_(chain_joints),
         composed_chains_(composed_chains),  tolerance_(tolerance) {}

    gtsam::NonlinearFactorGraph chainFactors(const Robot &robot, const int t, const boost::optional<PointOnLinks> &contact_points = boost::none) const;

    ~LeanDynamicsGraph() {}

    /**
     * Return nonlinear factor graph of all dynamics factors
     * @param robot          the robot
     * @param t              time step
     * link and 0 denotes no contact.
     * @param contact_points optional vector of contact points.
     * @param mu             optional coefficient of static friction.
     */
    gtsam::NonlinearFactorGraph dynamicsFactorGraph(
        const Robot &robot, const int t,
        const boost::optional<PointOnLinks> &contact_points = boost::none,
        const boost::optional<double> &mu = boost::none) const override;

};


} // namespace gtdynamics