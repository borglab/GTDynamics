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

#include <OptimizerSetting.h>
#include <Robot.h>

#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cmath>
#include <iosfwd>
#include <vector>
#include <string>
#include <boost/optional.hpp>

namespace gtdynamics {

// TODO(aescontrela3, yetongumich): can we not use inline here?

/* Shorthand for F_i_j_t, for wrenches at j-th joint on the i-th link at time t.
 */
inline gtsam::LabeledSymbol WrenchKey(int i, int j, int t) {
  return gtsam::LabeledSymbol('F', i * 16 + j,
                              t);  // a hack here for a key with 3 numbers
}

/* Shorthand for C_i_t, for contact wrench on i-th link at time t.*/
inline gtsam::LabeledSymbol ContactWrenchKey(int i, int t) {
  return gtsam::LabeledSymbol('C', i, t);
}

/* Shorthand for T_j_t, for torque on the j-th joint at time t. */
inline gtsam::LabeledSymbol TorqueKey(int j, int t) {
  return gtsam::LabeledSymbol('T', j, t);
}

/* Shorthand for p_i_t, for COM pose on the i-th link at time t. */
inline gtsam::LabeledSymbol PoseKey(int i, int t) {
  return gtsam::LabeledSymbol('p', i, t);
}

/* Shorthand for V_i_t, for 6D link twist vector on the i-th link. */
inline gtsam::LabeledSymbol TwistKey(int i, int t) {
  return gtsam::LabeledSymbol('V', i, t);
}

/* Shorthand for A_i_t, for twist accelerations on the i-th link at time t. */
inline gtsam::LabeledSymbol TwistAccelKey(int i, int t) {
  return gtsam::LabeledSymbol('A', i, t);
}

/* Shorthand for q_j_t, for j-th joint angle at time t. */
inline gtsam::LabeledSymbol JointAngleKey(int j, int t) {
  return gtsam::LabeledSymbol('q', j, t);
}

/* Shorthand for v_j_t, for j-th joint velocity at time t. */
inline gtsam::LabeledSymbol JointVelKey(int j, int t) {
  return gtsam::LabeledSymbol('v', j, t);
}

/* Shorthand for a_j_t, for j-th joint acceleration at time t. */
inline gtsam::LabeledSymbol JointAccelKey(int j, int t) {
  return gtsam::LabeledSymbol('a', j, t);
}

/* Shorthand for dt_k, for duration for timestep dt_k during phase k. */
inline gtsam::LabeledSymbol PhaseKey(int k) {
  return gtsam::LabeledSymbol('t', 0, k);
}

/* Shorthand for t_t, time at time step t. */
inline gtsam::LabeledSymbol TimeKey(int t) {
  return gtsam::LabeledSymbol('t', 1, t);
}

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
    opt_.f_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.fa_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.cp_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.cv_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.ca_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.planar_cost_model = gtsam::noiseModel::Constrained::All(3);
    opt_.prior_q_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_qv_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_qa_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.prior_t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.q_col_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.v_col_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.time_cost_model = gtsam::noiseModel::Constrained::All(1);
  }

  DynamicsGraph(const OptimizerSetting& opt) : opt_(opt)
  {
  }

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
  static gtsam::GaussianFactorGraph linearDynamicsGraph(const Robot &robot, const int t,
                                                        const Robot::JointValues& joint_angles,
                                                        const Robot::JointValues& joint_vels,
                                                        const Robot::FKResults &fk_results,
                                                        const boost::optional<gtsam::Vector3> &gravity = boost::none,
                                                        const boost::optional<gtsam::Vector3> &planar_axis = boost::none);

  /* return linear factor graph with priors on torques */
  static gtsam::GaussianFactorGraph linearFDPriors(const Robot &robot,
                                                   const int t, 
                                                   const Robot::JointValues& torque_values);

  /** sovle forward kinodynamics using linear factor graph, return values of all variables
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
  static gtsam::Values linearSolveFD(const Robot &robot, const int t,
                                     const Robot::JointValues& joint_angles,
                                     const Robot::JointValues& joint_vels,
                                     const Robot::JointValues& torques,
                                     const Robot::FKResults &fk_results,
                                     const boost::optional<gtsam::Vector3> &gravity = boost::none,
                                     const boost::optional<gtsam::Vector3> &planar_axis = boost::none);


  /* return q-level nonlinear factor graph (pose related factors) */
  gtsam::NonlinearFactorGraph qFactors(const Robot &robot,
                                       const int t) const;

  /* return v-level nonlinear factor graph (twist related factors) */
  gtsam::NonlinearFactorGraph vFactors(const Robot &robot,
                                       const int t) const;

  /* return a-level nonlinear factor graph (acceleration related factors) */
  gtsam::NonlinearFactorGraph aFactors(const Robot &robot,
                                       const int t) const;

  /* return dynamics-level nonlinear factor graph (wrench related factors) */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

  /** return nonlinear factor graph of all dynamics factors
  * Keyword arguments:
     robot                -- the robot
     t                    -- time step
     gravity              -- gravity in world frame
     planar_axis          -- axis of the plane, used only for planar robot
     contacts             -- vector of length num_links where 1 denotes
        contact link and 0 denotes no contact.
   */
  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none,
      const boost::optional<std::vector<uint>> &contacts = boost::none) const;

  /** return prior factors of torque, angle, velocity
  * Keyword arguments:
     robot                -- the robot
     t                    -- time step
     joint_angles         -- joint angles specified in order of joints
     joint_vels           -- joint velocites specified in order of joints
     torques              -- joint torques specified in order of joints
   */
  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const Robot &robot, const int t,
      const gtsam::Vector &joint_angles, const gtsam::Vector &joint_vels,
      const gtsam::Vector &torques) const;

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
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none) const;

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
      const std::vector<Robot> &robots,
      const std::vector<int> &phase_steps,
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
  gtsam::NonlinearFactorGraph
  targetAngleFactors(const Robot &robot, const int t,
                     const std::string &joint_name,
                     const double target_angle) const;

  /** return goal factors of link pose
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step to specify the goal
     link_name           -- name of the link to specify the goal
     target_pose         -- target link pose
   */
  gtsam::NonlinearFactorGraph
  targetPoseFactors(const Robot &robot, const int t,
                    const std::string &link_name,
                    const gtsam::Pose3 &target_pose) const;

  /** return the joint accelerations
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
   */
  static gtsam::Vector jointAccels(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint velocities. */
  static gtsam::Vector jointVels(const Robot &robot,
                                 const gtsam::Values &result, const int t);

  /* return joint angles. */
  static gtsam::Vector jointAngles(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint torques. */
  static gtsam::Vector jointTorques(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /** return the joint accelerations as std::map<name, acceleration>
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
   */
  static Robot::JointValues jointAccelsMap(const Robot &robot,
                                   const gtsam::Values &result, const int t);

  /** return zero values for all variables for initial value of optimization
  * Keyword arguments:
     robot               -- the robot
     t                   -- time step
   */
  static gtsam::Values zeroValues(const Robot &robot, const int t);

  /** return zero values of the trajectory for initial value of optimization
  * Keyword arguments:
     robot               -- the robot
     num_steps           -- total time steps
     num_phases          -- number of phases, -1 for not using
  multi-phase
   */
  static gtsam::Values zeroValuesTrajectory(const Robot &robot,
                                            const int num_steps,
                                            const int num_phases = -1);

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
                        const gtsam::Values &values,
                        const Robot &robot, const int t,
                        bool radial = false);

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
                                  const Robot &robot,
                                  const int num_steps, bool radial = false);

  /* return the optimizer setting. */
  const OptimizerSetting& opt() const
  {
    return opt_;
  }
};

}  // namespace gtdynamics
