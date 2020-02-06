/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @Author:Yetong Zhang
 */
#pragma once

#include <DynamicsGraph.h>
#include <Robot.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

#include <boost/optional.hpp>

namespace gtdynamics {
/**
 * Simulator is a class which simulate robot arm motion using forward
 * dynamics
 */
class Simulator {
 private:
  Robot robot_;
  int t_;
  DynamicsGraph graph_builder_;
  Robot::JointValues initial_angles_, initial_vels_;
  boost::optional<gtsam::Vector3> gravity_;
  boost::optional<gtsam::Vector3> planar_axis_;
  Robot::JointValues qs_, vs_, as_;
  gtsam::Values results_;

 public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- Simulator time step
   *  robot                    -- robotic robot
   *  initial_angels           -- initial joint angles
   *  initial_vels             -- initial joint velocities
   */
  Simulator(const Robot &robot,
            const Robot::JointValues &initial_angles,
            const Robot::JointValues &initial_vels,
            const boost::optional<gtsam::Vector3> &gravity = boost::none,
            const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
      : robot_(robot),
        t_(0),
        graph_builder_(DynamicsGraph()),
        initial_angles_(initial_angles),
        initial_vels_(initial_vels),
        gravity_(gravity),
        planar_axis_(planar_axis) {
    reset();
  }
  ~Simulator() {}

  /* reset simulation. */
  void reset(const double t = 0) {
    t_ = t;
    qs_ = initial_angles_;
    vs_ = initial_vels_;
    as_ = Robot::JointValues();
    results_ = gtsam::Values();
  }

  /**
   * perform forward dynamics to calculate accelerations, update a_, add new
   * values to results_ Keyword arguments: torques                   -- torques
   * for the time step
   */
  void forwardDynamics(const Robot::JointValues &torques) {
    
    auto fk_results = robot_.forwardKinematics(qs_, vs_);
    gtsam::Values result = graph_builder_.linearSolveFD(robot_, t_, qs_, vs_, torques, fk_results, gravity_, planar_axis_);
    results_.insert(result);

    // update accelerations
    as_ = DynamicsGraph::jointAccelsMap(robot_, result, t_);
  }

  /**
   * integrate to calculate new q, v for one time step, update q_, v_
   * Keyword arguments:
   *  torques                   -- torques for the time step
   *  dt                        -- duration for the time step
   */
  void integration(const double dt) {
    Robot::JointValues vs_new, qs_new;
    for (JointSharedPtr joint : robot_.joints())
    {
        std::string name = joint->name();
        vs_new[name] = vs_.at(name) + dt * as_.at(name);
        qs_new[name] = qs_.at(name) + dt * vs_.at(name) + 0.5 * as_.at(name) * std::pow(dt, 2);
    }
    vs_ = vs_new;
    qs_ = qs_new;
  }

  /**
   * simulate for one time step, update q_, v_, a_, t_, add the new values into
   * result_ Keyword arguments: torques                   -- torques for the
   * time step dt                        -- duration for the time step
   */
  void step(const Robot::JointValues &torques, const double dt) {
    forwardDynamics(torques);
    integration(dt);
    t_++;
  }

  /* simulation for the specified sequence of torques */
  gtsam::Values simulate(const std::vector<Robot::JointValues> torques_seq,
                         const double dt) {
    for (const Robot::JointValues &torques : torques_seq) {
      step(torques, dt);
    }
    return results_;
  }

  /* return joint angle values. */
  const Robot::JointValues &getJointAngles() const { return qs_; }

  /* return joint velocity values. */
  const Robot::JointValues &getJointVelocities() const { return vs_; }

  /* return joint acceleration values. */
  const Robot::JointValues &getJointAccelerations() const { return as_; }

  /* return all values during simulation. */
  const gtsam::Values &getValues() const { return results_; }
};

}  // namespace gtdynamics
