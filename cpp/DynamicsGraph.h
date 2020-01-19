/**
 * @file  DynamicsGraphBuilder.h
 * @brief robot arm motion planner using nonlinear factor graph
 * @Author: Yetong Zhang, Alejandro Escontrela
 */
#pragma once

#include <JointLimitFactor.h>
#include <JsonSaver.h>
#include <OptimizerSetting.h>
#include <PoseFactor.h>
#include <ToolPoseFactor.h>
#include <TorqueFactor.h>
#include <TwistAccelFactor.h>
#include <TwistFactor.h>
#include <WrenchEquivalenceFactor.h>
#include <WrenchFactors.h>
#include <WrenchPlanarFactor.h>
#include <cmath>
#include <gtsam/linear/NoiseModel.h>
#include <iostream>
#include <utils.h>

#include <RobotJoint.h>
#include <RobotLink.h>
#include <UniversalRobot.h>

#include <boost/optional.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

#include <fstream>
#include <iostream>

namespace robot
{

/* Shorthand for F_i_j_t, for wrenches at j-th joint on the i-th link at time t.
 */
gtsam::LabeledSymbol WrenchKey(int i, int j, int t)
{
  return gtsam::LabeledSymbol('F', i * 16 + j,
                              t); // a hack here for a key with 3 numbers
}

/* Shorthand for T_j_t, for torque on the j-th joint at time t. */
gtsam::LabeledSymbol TorqueKey(int j, int t)
{
  return gtsam::LabeledSymbol('T', j, t);
}

/* Shorthand for p_i_t, for COM pose on the i-th link at time t. */
gtsam::LabeledSymbol PoseKey(int i, int t)
{
  return gtsam::LabeledSymbol('p', i, t);
}

/* Shorthand for V_i_t, for 6D link twist vector on the i-th link. */
gtsam::LabeledSymbol TwistKey(int i, int t)
{
  return gtsam::LabeledSymbol('V', i, t);
}

/* Shorthand for A_i_t, for twist accelerations on the i-th link at time t. */
gtsam::LabeledSymbol TwistAccelKey(int i, int t)
{
  return gtsam::LabeledSymbol('A', i, t);
}

/* Shorthand for q_j_t, for j-th joint angle at time t. */
gtsam::LabeledSymbol JointAngleKey(int j, int t)
{
  return gtsam::LabeledSymbol('q', j, t);
}

/* Shorthand for v_j_t, for j-th joint velocity at time t. */
gtsam::LabeledSymbol JointVelKey(int j, int t)
{
  return gtsam::LabeledSymbol('v', j, t);
}

/* Shorthand for a_j_t, for j-th joint acceleration at time t. */
gtsam::LabeledSymbol JointAccelKey(int j, int t)
{
  return gtsam::LabeledSymbol('a', j, t);
}

/* Shorthand for t_k, for duration for timestep dt_k. */
gtsam::LabeledSymbol TimeKey(int k) { return gtsam::LabeledSymbol('t', 0, k); }

/**
 * DynamicsGraphBuilder is a class which builds a factor graph to do kinodynamic
 * motion planning
 */
class DynamicsGraphBuilder
{
private:
  manipulator::OptimizerSetting opt_;

public:
  /**
   * Constructor
   */
  explicit DynamicsGraphBuilder()
  {
    opt_ = manipulator::OptimizerSetting();
    // set all dynamics related factors to be constrained
    opt_.bp_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.bv_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.ba_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.p_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.v_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.a_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.f_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.tf_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.q_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.qv_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.setJointLimitCostModel(1e-3);

    opt_.setLM();
  }
  ~DynamicsGraphBuilder() {}

  enum CollocationScheme
  {
    Euler,
    RungeKutta,
    Trapezoidal,
    HermiteSimpson
  };

  enum OptimizerType
  {
    GaussNewton,
    LM,
    PDL
  };

  /** return nonlinear factor graph of all dynamics factors
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     gravity                    -- gravity in world frame
     plannar_axis               -- axis of the plane, used only for planar robot
   */
  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const UniversalRobot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &plannar_axis = boost::none) const;

  /** return nonlinear factor graph of the entire trajectory
  * Keyword arguments:
     robot                      -- the robot
     num_steps                  -- total time steps
     dt                         -- duration of each time step
     collocation                -- the collocation scheme
     gravity                    -- gravity in world frame
     plannar_axis               -- axis of the plane, used only for planar robot
   */
  gtsam::NonlinearFactorGraph trajectoryFG(
      const UniversalRobot &robot, const int num_steps, const double dt,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &plannar_axis = boost::none) const;

  /** return nonlinear factor graph of the entire trajectory for multi-phase
  * Keyword arguments:
     robot                      -- the robot
     phase_steps                -- number of time steps for each phase
     collocation                -- the collocation scheme
     gravity                    -- gravity in world frame
     plannar_axis               -- axis of the plane, used only for planar robot
   */
  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const UniversalRobot &robot, const std::vector<int> &phase_steps,
      const CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &plannar_axis = boost::none) const;

  /** return collocation factors on angles and velocities from time step t to t+1
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     dt                         -- duration of each timestep
     collocation                -- collocation scheme chosen
   */
  gtsam::ExpressionFactorGraph collocationFactors(const UniversalRobot &robot,
                                                  const int t, const double dt,
                                                  const CollocationScheme collocation) const;


  /** return collocation factors on angles and velocities from time step t to t+1, with dt as a varaible
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     phase                      -- the phase of the timestep
     collocation                -- collocation scheme chosen
   */
  gtsam::ExpressionFactorGraph multiPhaseCollocationFactors(const UniversalRobot &robot,
                                                  const int t, const int phase,
                                                  const CollocationScheme collocation) const;


  /** return joint factors to limit angle, velocity, acceleration, and torque
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
   */
  gtsam::NonlinearFactorGraph jointLimitFactors(const UniversalRobot &robot,
                                                const int t) const;

  /** return prior factors of torque, angle, velocity
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
     joint_angles               -- joint angles specified in order of joints
     joint_vels                 -- joint velocites specified in order of joints
     torques                    -- joint torques specified in order of joints
   */
  gtsam::NonlinearFactorGraph
  forwardDynamicsPriors(const UniversalRobot &robot, const int t,
                        const gtsam::Vector &joint_angles,
                        const gtsam::Vector &joint_vels,
                        const gtsam::Vector &torques) const;

  /** return prior factors of initial state, torques along trajectory
  * Keyword arguments:
     robot                      -- the robot
     num_steps                  -- total time steps
     joint_angles               -- joint angles specified in order of joints
     joint_vels                 -- joint velocites specified in order of joints
     torques_seq                -- joint torques along the trajectory
   */
  gtsam::NonlinearFactorGraph
  trajectoryFDPriors(const UniversalRobot &robot, const int num_steps,
                     const gtsam::Vector &joint_angles,
                     const gtsam::Vector &joint_vels,
                     const std::vector<gtsam::Vector> &torques_seq) const;

  /** return the joint accelerations
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
   */
  static gtsam::Vector jointAccels(const UniversalRobot &robot,
                                   const gtsam::Values &result, const int t);

  static gtsam::Vector jointVels(const UniversalRobot &robot,
                                 const gtsam::Values &result, const int t);

  static gtsam::Vector jointAngles(const UniversalRobot &robot,
                                   const gtsam::Values &result, const int t);

  /** return zero values for all variables for initial value of optimization
  * Keyword arguments:
     robot                      -- the robot
     t                          -- time step
   */
  static gtsam::Values zeroValues(const UniversalRobot &robot, const int t);

  /** return zero values of the trajectory for initial value of optimization
  * Keyword arguments:
     robot                      -- the robot
     num_steps                  -- total time steps
     num_phases                 -- number of phases, -1 for not using multi-phase
   */
  static gtsam::Values zeroValuesTrajectory(const UniversalRobot &robot, const int num_steps, const int num_phases=-1);

  /** optimize factor graph
  * Keyword arguments:
     graph                      -- nonlinear factor graph 
     init_values                -- initial values for optimization
     optim_type                 -- choice of optimizer type
   */
  gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph &graph,
      const gtsam::Values &init_values, OptimizerType optim_type) const;

  // print the factors of the factor graph
  static void print_graph(const gtsam::NonlinearFactorGraph &graph);

  // print the values
  static void print_values(const gtsam::Values &values);

  /** save factor graph in json format for visualization
  * Keyword arguments:
     file_path                  -- path of the json file to store the graph
     graph                      -- factor graph
     values                     -- values of variables in factor graph
     robot                      -- the robot
     t                          -- time step
     radial                     -- option to display in radial format
   */
  static void saveGraph(const std::string &file_path,
                        const gtsam::NonlinearFactorGraph &graph,
                        const gtsam::Values &values,
                        const UniversalRobot &robot, const int t,
                        bool radial = false);
};

} // namespace robot
