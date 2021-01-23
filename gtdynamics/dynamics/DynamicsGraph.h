/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsGraph.h
 * @brief Builds a dynamics graph from a Robot object.
 * @author Yetong Zhang, Alejandro Escontrela
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <cmath>
#include <iosfwd>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {

//TODO(aescontrela3, yetongumich): can we not use inline here?

/* Shorthand for C_i_k_t, for kth contact wrench on i-th link at time t.*/
inline DynamicsSymbol ContactWrenchKey(int i, int k, int t) {
  return DynamicsSymbol::LinkJointSymbol("C", i, k, t);
}

/* Shorthand for dt_k, for duration for timestep dt_k during phase k. */
inline DynamicsSymbol PhaseKey(int k) {
  return DynamicsSymbol::SimpleSymbol("dt", k);
}

/* Shorthand for t_t, time at time step t. */
inline DynamicsSymbol TimeKey(int t) {
  return DynamicsSymbol::SimpleSymbol("t", t);
}

/**
 * ContactPoint defines a single contact point at a link.
 *
 * @param point The location of the contact point relative to the link COM.
 * @param id Each link's contact points must have a unique contact id.
 * @param height Height at which contact is made.
 */
struct ContactPoint {
  gtsam::Point3 point;
  int id;
  double height = 0.0;

  bool operator==(const ContactPoint &other) {
    return (point == other.point &&
            id == other.id &&
            height == other.height);
  }
  bool operator!=(const ContactPoint &other) {
    return !(*this == other);
  }
};

///< Map of link name to ContactPoint
using ContactPoints = std::map<std::string, ContactPoint>;

/**
 * DynamicsGraph is a class which builds a factor graph to do kinodynamic
 * motion planning
 */
class DynamicsGraph {
 private:
  OptimizerSetting opt_;

 public:
  /**
   * Constructor
   */
  DynamicsGraph() : opt_(OptimizerSetting()) {}

  explicit DynamicsGraph(const OptimizerSetting &opt) : opt_(opt) {}

  ~DynamicsGraph() {}

  enum CollocationScheme { Euler, RungeKutta, Trapezoidal, HermiteSimpson };

  /**
   * Return linear factor graph of all dynamics factors
   * @param robot        the robot
   * @param t            time step
   * @param joint_angles joint angles
   * @param joint_vels   joint velocities
   * @param fk_results   forward kinematics results
   * @param gravity      gravity in world frame
   * @param planar_axis  axis of the plane, used only for planar robot
   */
  static gtsam::GaussianFactorGraph linearDynamicsGraph(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /// Return linear factor graph with priors on torques
  static gtsam::GaussianFactorGraph linearFDPriors(
      const Robot &robot, const int t, const JointValues &torque_values);

  /// Return linear factor graph with priors on joint accelerations
  static gtsam::GaussianFactorGraph linearIDPriors(
      const Robot &robot, const int t, const JointValues &joint_accels);

  /**
   * Solve forward kinodynamics using linear factor graph.
   *
   * @param robot        the robot
   * @param t            time step
   * @param joint_angles std::map <joint name, angle>
   * @param joint_vels   std::map <joint name, velocity>
   * @param torques      std::map <joint name, torque>
   * @param fk_results   forward kinematics results
   * @param gravity      gravity in world frame
   * @param planar_axis  axis of the plane, used only for planar robot
   * @return values of all variables
   */
  static gtsam::Values linearSolveFD(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const JointValues &torques,
      const FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /**
   * Solve inverse kinodynamics using linear factor graph.
   * @param  robot        the robot
   * @param  t            time step
   * @param  joint_angles std::map <joint name, angle>
   * @param  joint_vels   std::map <joint name, velocity>
   * @param  torques      std::map <joint name, torque>
   * @param  fk_results   forward kinematics results
   * @param  gravity      gravity in world frame
   * @param  planar_axis  axis of the plane, used only for planar robot
   *
   * @return values of all variables
   */
  static gtsam::Values linearSolveID(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const JointValues &torques,
      const FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /// Return q-level nonlinear factor graph (pose related factors)
  gtsam::NonlinearFactorGraph qFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /// Return v-level nonlinear factor graph (twist related factors)
  gtsam::NonlinearFactorGraph vFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /// Return a-level nonlinear factor graph (acceleration related factors)
  gtsam::NonlinearFactorGraph aFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /// Return dynamics-level nonlinear factor graph (wrench related factors)
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /**
   * Return nonlinear factor graph of all dynamics factors
   * @param robot          the robot
   * @param t              time step
   * @param gravity        gravity in world frame
   * @param planar_axis    axis of the plane, used only for planar robot contact
   * link and 0 denotes no contact.
   * @param contact_points optional vector of contact points.
   * @param mu             optional coefficient of static friction.
   */
  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /**
   * Return prior factors of torque, angle, velocity
   * @param robot        the robot
   * @param t            time step
   * @param joint_angles joint angles specified in order of joints
   * @param joint_vels   joint velocities specified in order of joints
   * @param torques      joint torques specified in order of joints
   */
  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &torques) const;

  /**
   * Return prior factors of accel, angle, velocity
   * @param robot        the robot
   * @param t            time step
   * @param joint_angles joint angles specified in order of joints
   * @param joint_vels   joint velocities specified in order of joints
   * @param joint_accels joint accels specified in order of joints
   */
  gtsam::NonlinearFactorGraph inverseDynamicsPriors(
      const Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &joint_accels) const;

  /**
   * Return prior factors of torque, angle, velocity
   * @param robot        the robot
   * @param t            time step
   * @param joint_angles map from joint name to joint angle
   * @param joint_vels   map from joint name to joint velocity
   * @param torques      map from joint name to joint torque
   */
  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const JointValues &torques) const;

  /**
   * Return prior factors of initial state, torques along trajectory
   * @param robot        the robot
   * @param num_steps    total time steps
   * @param joint_angles joint angles specified in order of joints
   * @param joint_vels   joint velocities specified in order of joints
   * @param torques_seq  joint torques along the trajectory
   */
  gtsam::NonlinearFactorGraph trajectoryFDPriors(
      const Robot &robot, const int num_steps,
      const gtsam::Vector &joint_angles, const gtsam::Vector &joint_vels,
      const std::vector<gtsam::Vector> &torques_seq) const;

  /**
   * Return nonlinear factor graph of the entire trajectory
   * @param robot       the robot
   * @param num_steps   total time steps
   * @param dt          duration of each time step
   * @param collocation the collocation scheme
   * @param gravity     gravity in world frame
   * @param planar_axis axis of the plane, used only for planar robot
   */
  gtsam::NonlinearFactorGraph trajectoryFG(
      const Robot &robot, const int num_steps, const double dt,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /**
   * Return nonlinear factor graph of the entire trajectory for multi-phase
   * @param robots            the robot configuration for each phase
   * @param phase_steps       number of time steps for each phase
   * @param transition_graphs transition step graphs with guardian factors
   * @param collocation       the collocation scheme
   * @param gravity           gravity in world frame
   * @param planar_axis       axis of the plane, only for planar robot
   */
  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const std::vector<Robot> &robots, const std::vector<int> &phase_steps,
      const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<std::vector<ContactPoints>> &phase_contact_points =
          boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /**
   * Return collocation factors on angles and velocities from time step t to t+1
   * @param robot       the robot
   * @param t           time step
   * @param dt          duration of each timestep
   * @param collocation collocation scheme chosen
   */
  gtsam::NonlinearFactorGraph collocationFactors(
      const Robot &robot, const int t, const double dt,
      const CollocationScheme collocation) const;

  /**
   * Return collocation factors on angles and velocities from time step t to
   * t+1, with dt as a varaible
   * @param robot       the robot
   * @param t           time step
   * @param phase       the phase of the timestep
   * @param collocation collocation scheme chosen
   */
  gtsam::NonlinearFactorGraph multiPhaseCollocationFactors(
      const Robot &robot, const int t, const int phase,
      const CollocationScheme collocation) const;

  /**
   * Return joint factors to limit angle, velocity, acceleration, and torque
   * @param robot the robot
   * @param t time step
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(const Robot &robot,
                                                const int t) const;

  /**
   * Return goal factors of joint angle
   * @param robot        the robot
   * @param t            time step to specify the goal
   * @param joint_name   name of the joint to specify the goal
   * @param target_angle target joint angle
   */
  gtsam::NonlinearFactorGraph targetAngleFactors(
      const Robot &robot, const int t, const std::string &joint_name,
      const double target_angle) const;

  /**
   * Return goal factors of link pose
   * @param robot       the robot
   * @param t           time step to specify the goal
   * @param link_name   name of the link to specify the goal
   * @param target_pose target link pose
   */
  gtsam::NonlinearFactorGraph targetPoseFactors(
      const Robot &robot, const int t, const std::string &link_name,
      const gtsam::Pose3 &target_pose) const;

  /**
   * Return the joint accelerations
   * @param robot the robot
   * @param t     time step
   */
  static gtsam::Vector jointAccels(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /// Return joint velocities.
  static gtsam::Vector jointVels(const Robot &robot,
                                 const gtsam::Values &result, const int t);

  /// Return joint angles.
  static gtsam::Vector jointAngles(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /// Return joint torques.
  static gtsam::Vector jointTorques(const Robot &robot,
                                    const gtsam::Values &result, const int t);

  /**
   * Return the joint accelerations as std::map<name, acceleration>
   * @param robot the robot
   * @param t     time step
   */
  static JointValues jointAccelsMap(const Robot &robot,
                                    const gtsam::Values &result, const int t);

  /// Return joint velocities as std::map<name, velocity>.
  static JointValues jointVelsMap(const Robot &robot,
                                  const gtsam::Values &result, const int t);

  /// Return joint angles as std::map<name, angle>.
  static JointValues jointAnglesMap(const Robot &robot,
                                    const gtsam::Values &result, const int t);

  /// Return joint torques as std::map<name, torque>.
  static JointValues jointTorquesMap(const Robot &robot,
                                     const gtsam::Values &result, const int t);

  /// Rrint the factors of the factor graph
  static void printGraph(const gtsam::NonlinearFactorGraph &graph);

  /// Print the values
  static void printValues(const gtsam::Values &values);

  /**
   * Save factor graph in json format for visualization
   * @param file_path path of the json file to store the graph
   * @param graph     factor graph
   * @param values    values of variables in factor graph
   * @param robot     the robot
   * @param t         time step
   * @param radial    option to display in radial format
   */
  static void saveGraph(const std::string &file_path,
                        const gtsam::NonlinearFactorGraph &graph,
                        const gtsam::Values &values, const Robot &robot,
                        const int t, bool radial = false);

  /**
   * Save factor graph of multiple time steps in json format
   * @param file_path path of the json file to store the graph
   * @param graph     factor graph
   * @param values    values of variables in factor graph
   * @param robot     the robot
   * @param num_steps number of time steps
   * @param radial    option to display in radial format
   */
  static void saveGraphMultiSteps(const std::string &file_path,
                                  const gtsam::NonlinearFactorGraph &graph,
                                  const gtsam::Values &values,
                                  const Robot &robot, const int num_steps,
                                  bool radial = false);

  /**
   * Save factor graph of trajectory in json format
   * @param file_path path of the json file to store the graph
   * @param graph     factor graph
   * @param values    values of variables in factor graph
   * @param robot     the robot
   * @param num_steps number of time steps
   * @param radial    option to display in radial format
   */
  static void saveGraphTraj(const std::string &file_path,
                            const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &values, const int num_steps);

  /// Return the optimizer setting.
  const OptimizerSetting &opt() const { return opt_; }
};

}  // namespace gtdynamics
