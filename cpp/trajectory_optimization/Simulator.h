/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @Author:Yetong Zhang
 */
#pragma once

#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>

namespace robot
{
/**
 * Simulator is a class which simulate robot arm motion using forward
 * dynamics
 */
class Simulator
{
private:
  UniversalRobot robot_;
  int t_;
  DynamicsGraphBuilder graph_builder_;
  gtsam::Vector initial_angles_, initial_vels_;
  boost::optional<gtsam::Vector3> gravity_;
  boost::optional<gtsam::Vector3> planar_axis_;
  gtsam::Vector qs_, vs_, as_;
  gtsam::Values results_;

public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- Simulator time step
   *  robot                    -- robotic manipulator
   *  initialJointAngles       -- initial joint angles
   *  initialJointVelocities   -- initial joint velocities
   *  initialConditions        -- condition to specify for the first timestep
   */
  Simulator(
      const UniversalRobot &robot, const gtsam::Vector &initialJointAngles,
      const gtsam::Vector &initialJointVelocities,
      //  const boost::optional<gtsam::Values> &initialConditions = boost::none,
      const boost::optional<gtsam::Vector3> &gravity = boost::none,
      const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
      : robot_(robot), t_(0), graph_builder_(DynamicsGraphBuilder()),
        initial_angles_(initialJointAngles), initial_vels_(initialJointVelocities),
        gravity_(gravity), planar_axis_(planar_axis)
  {
    reset();
  }
  ~Simulator() {}

  /* reset simulation. */
  void reset()
  {
    t_ = 0;
    qs_ = initial_angles_;
    vs_ = initial_vels_;
    as_ = gtsam::Vector::Zero(robot_.numJoints());
  }

  /**
   * perform forward dynamics to calculate accelerations, update a_, add new values to results_
   * Keyword arguments:
   *  torques                   -- torques for the time step
   */
  void forwardDynamics(const gtsam::Vector &torques)
  {
    // construct Dynamics Factor Graph
    gtsam::NonlinearFactorGraph graph =
        graph_builder_.dynamicsFactorGraph(robot_, t_, gravity_, planar_axis_);
    graph.add(graph_builder_.forwardDynamicsPriors(robot_, t_, qs_, vs_, torques));

    // optimize factor graph
    gtsam::Values init_values = DynamicsGraphBuilder::zeroValues(robot_, t_);
    gtsam::Values result = graph_builder_.optimize(graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);
    results_.insert(result);

    // update values
    as_ = DynamicsGraphBuilder::jointAccels(robot_, result, t_);
  }

  /**
   * integrate to calculate new q, v for one time step, update q_, v_
   * Keyword arguments:
   *  torques                   -- torques for the time step
   *  dt                        -- duration for the time step
   */
  void integration(const double dt)
  {
    gtsam::Vector vs_new = (vs_ + dt * as_).eval();
    gtsam::Vector qs_new = (qs_ + dt * vs_ + 0.5 * as_ * std::pow(dt, 2)).eval();
    vs_ = vs_new;
    qs_ = qs_new;
  }

  /**
   * simulate for one time step, update q_, v_, a_, t_, add the new values into result_
   * Keyword arguments:
   *  torques                   -- torques for the time step
   *  dt                        -- duration for the time step
   */
  void step(const gtsam::Vector &torques, const double dt)
  {
    forwardDynamics(torques);
    integration(dt);
    t_++;
  }

  /* simulation for the specified sequence of torques */
  gtsam::Values simulate(const std::vector<gtsam::Vector> torques_seq, const double dt)
  {
    for (const gtsam::Vector &torques : torques_seq)
    {
      step(torques, dt);
    }
    return results_;
  }

  /* return joint angle values. */
  const gtsam::Vector &getJointAngles() const { return qs_; }

  /* return joint velocity values. */
  const gtsam::Vector &getJointVelocities() const { return vs_; }

  /* return joint acceleration values. */
  const gtsam::Vector &getJointAccelerations() const { return as_; }

  /* return all values during simulation. */
  const gtsam::Values &getValues() const { return results_; }
};

} // namespace robot
