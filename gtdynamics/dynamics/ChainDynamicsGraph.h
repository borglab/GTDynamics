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
  double angle_tolerance_;
  double torque_tolerance_;
  double dynamics_tolerance_;
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
       const double &angle_tolerance, const double &torque_tolerance,
       const double &dynamics_tolerance,
       const boost::optional<Vector3> &gravity = boost::none,
       const boost::optional<Vector3> &planar_axis = boost::none)
       : DynamicsGraph(opt, gravity, planar_axis),
         chain_joints_(getChainJoints(robot)),
         composed_chains_(getComposedChains(chain_joints_)),
         angle_tolerance_(angle_tolerance), 
         torque_tolerance_(torque_tolerance),
         dynamics_tolerance_(dynamics_tolerance),
         trunk_mass_(robot.link("trunk")->mass()) {}

    // Destructor
    ~ChainDynamicsGraph() {}

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
       const boost::optional<PointOnLinks> &contact_points = boost::none,
       const boost::optional<double> &mu = boost::none) const override;

  // Get a vector of legs, each leg is a vector of its joints
  static std::vector<std::vector<JointSharedPtr>> getChainJoints(
      const Robot &robot);

  // Get composed chains from chain joints
  static std::vector<Chain> getComposedChains(
      std::vector<std::vector<JointSharedPtr>> &chain_joints);

};


} // namespace gtdynamics