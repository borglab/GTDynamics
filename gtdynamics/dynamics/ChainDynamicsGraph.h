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


//#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/dynamics/Chain.h"
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include "gtdynamics/factors/ContactDynamicsMomentFactor.h"
#include "gtdynamics/factors/WrenchFactor.h"
#include <gtdynamics/factors/ContactHeightFactor.h>

namespace gtdynamics {

using gtsam::Vector3;
using gtsam::Vector3_;
using gtsam::Vector6;
using gtsam::Vector6_;
using gtsam::Double_;
using gtsam::Pose3;
using gtsam::Point3;
using gtsam::Rot3;
using gtsam::Key;
using gtsam::NonlinearFactorGraph ;

class ChainDynamicsGraph : public DynamicsGraph {

  private:
  std::vector<std::vector<JointSharedPtr>> chain_joints_;
  std::vector<Chain> composed_chains_;
  double trunk_mass_;

  public:

    /**
     * Constructor
     * @param  robot      the robot
     * @param  opt          settings for optimizer
     * @param  angle_tolerance   tolerance for angle estimation
     * @param  torque_tolerance   tolerance for torque estimation
     * @param  dynamics_tolerance   tolerance for dynamics estimation
     * @param  gravity      gravity in world frame
     * @param  planar_axis  axis of the plane, used only for planar robot
     */
   ChainDynamicsGraph(
       const Robot &robot,
       const OptimizerSetting &opt,
       const std::optional<Vector3> &gravity = {},
       const std::optional<Vector3> &planar_axis = {})
       : DynamicsGraph(opt, gravity, planar_axis),
         chain_joints_(getChainJoints(robot)),
         composed_chains_(getComposedChains(chain_joints_)),
         trunk_mass_(robot.link("trunk")->mass()) {}

    // Destructor
    ~ChainDynamicsGraph() {}


  /// Return q-level nonlinear factor graph (pose related factors)
    gtsam::NonlinearFactorGraph qFactors(
        const Robot &robot, const int t,
        const std::optional<PointOnLinks> &contact_points = {}) const override;

   /**
    * Create dynamics factors of the chain graph
    * @param robot          the robot
    * @param t              time step
    * link and 0 denotes no contact.
    * @param contact_points optional vector of contact points.
    * @param mu             optional coefficient of static friction.
    */
   NonlinearFactorGraph dynamicsFactors(
       const Robot &robot, const int t,
       const std::optional<PointOnLinks> &contact_points = {},
       const std::optional<double> &mu = {}) const override;

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
      const std::optional<PointOnLinks> &contact_points = {},
      const std::optional<double> &mu = {}) const override;

  // Get a vector of legs, each leg is a vector of its joints
  static std::vector<std::vector<JointSharedPtr>> getChainJoints(
      const Robot &robot);

  // Get composed chains from chain joints
  static std::vector<Chain> getComposedChains(
      std::vector<std::vector<JointSharedPtr>> &chain_joints);

};


} // namespace gtdynamics