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

#ifndef GTDYNAMICS_DYNAMICS_DYNAMICSGRAPH_H_
#define GTDYNAMICS_DYNAMICS_DYNAMICSGRAPH_H_

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cmath>
#include <iosfwd>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {

// TODO(aescontrela3, yetongumich): can we not use inline here?

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
 * Keyword Arguments:
 *  name -- The name of the link in contact.
 *  contact_point -- The location of the contact point relative to the link
 *    Com.
 *  contact_id -- Each link's contact points must have a unique contact id.
 *  contact_height -- Height at which contact is made.
 */
typedef struct {
  std::string name;
  gtsam::Point3 contact_point;
  int contact_id;
  double contact_height = 0.0;
} ContactPoint;
typedef std::vector<ContactPoint> ContactPoints;

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
  DynamicsGraph() {
    opt_ = OptimizerSetting();
    // set all dynamics related factors to be constrained
    opt_.bp_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.bv_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.ba_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.p_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.v_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.a_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.linear_a_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.f_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.linear_f_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.fa_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.linear_t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.cp_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.cfriction_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.cv_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.ca_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.planar_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.linear_planar_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.prior_q_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_qv_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_qa_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.q_col_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.v_col_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.time_cost_model = gtsam::noiseModel::Constrained::All(1);
  }

  explicit DynamicsGraph(const OptimizerSetting &opt) : opt_(opt) {}

  ~DynamicsGraph() {}

  enum CollocationScheme { Euler, RungeKutta, Trapezoidal, HermiteSimpson };

  /** return linear factor graph of all dynamics factors
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
     joint_angles        -- joint angles
     joint_vels          -- joint velocities
     fk_results          -- forward kinematics results
     gravity             -- gravity in world frame
     planar_axis         -- axis of the plane, used only for planar robot
   */
  static gtsam::GaussianFactorGraph linearDynamicsGraph(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const Robot::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /* return linear factor graph with priors on torques */
  static gtsam::GaussianFactorGraph linearFDPriors(
      const Robot &robot, const int t, const JointValues &torque_values);

  /* return linear factor graph with priors on joint accelerations */
  static gtsam::GaussianFactorGraph linearIDPriors(
      const Robot &robot, const int t, const JointValues &joint_accels);

  /** sovle forward kinodynamics using linear factor graph, return values of all
  variables
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
     joint_angles        -- std::map <joint name, angle>
     joint_vels          -- std::map <joint name, velocity>
     torques             -- std::map <joint name, torque>
     fk_results          -- forward kinematics results
     gravity             -- gravity in world frame
     planar_axis         -- axis of the plane, used only for planar robot
  * return values of all variables
  */
  static gtsam::Values linearSolveFD(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const JointValues &torques,
      const Robot::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /** sovle inverse kinodynamics using linear factor graph, return values of all
  variables
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
     joint_angles        -- std::map <joint name, angle>
     joint_vels          -- std::map <joint name, velocity>
     joint_accels        -- std::map <joint name, acceleration>
     fk_results          -- forward kinematics results
     gravity             -- gravity in world frame
     planar_axis         -- axis of the plane, used only for planar robot
  * return values of all variables
  */
  static gtsam::Values linearSolveID(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels, const JointValues &joint_accels,
      const Robot::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /* return q-level nonlinear factor graph (pose related factors) */
  gtsam::NonlinearFactorGraph qFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /* return v-level nonlinear factor graph (twist related factors) */
  gtsam::NonlinearFactorGraph vFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /* return a-level nonlinear factor graph (acceleration related factors) */
  gtsam::NonlinearFactorGraph aFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none) const;

  /* return dynamics-level nonlinear factor graph (wrench related factors) */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /** return nonlinear factor graph of all dynamics factors
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     gravity                    -- gravity in world frame
     planar_axis                -- axis of the plane, used only for planar
        robot contact link and 0 denotes no contact.
     contact_points             -- optional vector of contact points.
     mu                         -- optional coefficient of static friction.
   */
  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /** return prior factors of torque, angle, velocity
  * Keyword arguments:
     robot                -- the robot
     t                    -- time step
     joint_angles         -- joint angles specified in order of joints
     joint_vels           -- joint velocites specified in order of joints
     torques              -- joint torques specified in order of joints
   */
  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &torques) const;

  /** return prior factors of accel, angle, velocity
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     joint_angles               -- joint angles specified in order of joints
     joint_vels                 -- joint velocites specified in order of joints
     joint_accels               -- joint accels specified in order of joints
   */
  gtsam::NonlinearFactorGraph inverseDynamicsPriors(
      const Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &joint_accels) const;

  /** return prior factors of torque, angle, velocity
  * Keyword arguments:
     robot                -- the robot
     t                    -- time step
     joint_angles         -- map from joint name to joint angle
     joint_vels           -- map from joint name to joint velocity
     torques              -- map from joint name to joint torque
   */
  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const Robot &robot, const int t, const JointValues &joint_angles,
      const JointValues &joint_vels,
      const JointValues &torques) const;

  /** return prior factors of initial state, torques along trajectory
  * Keyword arguments:
     robot               -- the robot
     num_steps           -- total time steps
     joint_angles        -- joint angles specified in order of joints
     joint_vels          -- joint velocites specified in order of joints
     torques_seq         -- joint torques along the trajectory
   */
  gtsam::NonlinearFactorGraph trajectoryFDPriors(
      const Robot &robot, const int num_steps,
      const gtsam::Vector &joint_angles, const gtsam::Vector &joint_vels,
      const std::vector<gtsam::Vector> &torques_seq) const;

  /** return nonlinear factor graph of the entire trajectory
  * Keyword arguments:
     robot               -- the robot
     num_steps           -- total time steps
     dt                  -- duration of each time step
     collocation         -- the collocation scheme
     gravity             -- gravity in world frame
     planar_axis         -- axis of the plane, used only for planar robot
   */
  gtsam::NonlinearFactorGraph trajectoryFG(
      const Robot &robot, const int num_steps, const double dt,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<ContactPoints> &contact_points = boost::none,
      const boost::optional<double> &mu = boost::none) const;

  /** return nonlinear factor graph of the entire trajectory for multi-phase
  * Keyword arguments:
     robots              -- the robot configuration for each phase
     phase_steps         -- number of time steps for each phase
     transition_graphs   -- transition step graphs with guardian factors
     collocation         -- the collocation scheme
     gravity             -- gravity in world frame
     planar_axis         -- axis of the plane, only for planar robot
   */
  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const std::vector<Robot> &robots, const std::vector<int> &phase_steps,
      const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /** return collocation factors on angles and velocities from time step t to
  t+1
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
     dt                  -- duration of each timestep
     collocation         -- collocation scheme chosen
   */
  gtsam::NonlinearFactorGraph collocationFactors(
      const Robot &robot, const int t, const double dt,
      const CollocationScheme collocation) const;

  /** return collocation factors on angles and velocities from time step t to
  t+1, with dt as a varaible
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
     phase               -- the phase of the timestep
     collocation         -- collocation scheme chosen
   */
  gtsam::NonlinearFactorGraph multiPhaseCollocationFactors(
      const Robot &robot, const int t, const int phase,
      const CollocationScheme collocation) const;

  /** return joint factors to limit angle, velocity, acceleration, and torque
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(const Robot &robot,
                                                const int t) const;

  /** return goal factors of joint angle
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step to specify the goal
     joint_name          -- name of the joint to specify the goal
     target_angle        -- target joint angle
   */
  gtsam::NonlinearFactorGraph targetAngleFactors(
      const Robot &robot, const int t, const std::string &joint_name,
      const double target_angle) const;

  /** return goal factors of link pose
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step to specify the goal
     link_name           -- name of the link to specify the goal
     target_pose         -- target link pose
   */
  gtsam::NonlinearFactorGraph targetPoseFactors(
      const Robot &robot, const int t, const std::string &link_name,
      const gtsam::Pose3 &target_pose) const;

  /** return the joint accelerations as std::map<name, acceleration>
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
   */
  static JointValues jointAccels(const Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint velocities as std::map<name, velocity>. */
  static JointValues jointVels(const Robot &robot,
                                         const gtsam::Values &result,
                                         const int t);

  /* return joint angles as std::map<name, angle>. */
  static JointValues jointAngles(const Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint torques as std::map<name, torque>. */
  static JointValues jointTorques(const Robot &robot,
                                            const gtsam::Values &result,
                                            const int t);

  /* print the factors of the factor graph */
  static void printGraph(const gtsam::NonlinearFactorGraph &graph);

  /* print the values */
  static void printValues(const gtsam::Values &values);

  /** save factor graph in json format for visualization
  * Keyword arguments:
     file_path           -- path of the json file to store the graph
     graph               -- factor graph
     values              -- values of variables in factor graph
     robot               -- the robot
     t                   -- time step
     radial              -- option to display in radial format
   */
  static void saveGraph(const std::string &file_path,
                        const gtsam::NonlinearFactorGraph &graph,
                        const gtsam::Values &values, const Robot &robot,
                        const int t, bool radial = false);

  /** save factor graph of multiple time steps in json format
  * Keyword arguments:
     file_path           -- path of the json file to store the graph
     graph               -- factor graph
     values              -- values of variables in factor graph
     robot               -- the robot
     num_steps           -- number of time steps
     radial              -- option to display in radial format
   */
  static void saveGraphMultiSteps(const std::string &file_path,
                                  const gtsam::NonlinearFactorGraph &graph,
                                  const gtsam::Values &values,
                                  const Robot &robot, const int num_steps,
                                  bool radial = false);

  /** save factor graph of trajectory in json format
  * Keyword arguments:
     file_path           -- path of the json file to store the graph
     graph               -- factor graph
     values              -- values of variables in factor graph
     robot               -- the robot
     num_steps           -- number of time steps
     radial              -- option to display in radial format
   */
  static void saveGraphTraj(const std::string &file_path,
                            const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &values, const int num_steps);

  /* return the optimizer setting. */
  const OptimizerSetting &opt() const { return opt_; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_DYNAMICS_DYNAMICSGRAPH_H_
