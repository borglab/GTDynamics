/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {
/**
 * Simulator is a class which simulate robot arm motion using forward
 * dynamics.
 */
class Simulator {
 private:
  Robot robot_;
  int t_;
  DynamicsGraph graph_builder_;
  JointValues initial_angles_, initial_vels_;
  boost::optional<gtsam::Vector3> gravity_;
  boost::optional<gtsam::Vector3> planar_axis_;
  JointValues qs_, vs_, as_;
  gtsam::Values results_;

 public:
  /**
   * Constructor
   *
   * @param time_step      Simulator time step
   * @param robot          robotic robot
   * @param initial_angels initial joint angles
   * @param initial_vels   initial joint velocities
   * @param gravity        gravity vector
   * @param planar_axis    planar axis vector
   */
  Simulator(const Robot &robot, const JointValues &initial_angles,
            const JointValues &initial_vels,
            const boost::optional<gtsam::Vector3> &gravity = boost::none,
            const boost::optional<gtsam::Vector3> &planar_axis = boost::none)
      : robot_(robot),
        t_(0),
        graph_builder_(DynamicsGraph(gravity, planar_axis)),
        initial_angles_(initial_angles),
        initial_vels_(initial_vels) {
    reset();
  }
  ~Simulator() {}

  /// Reset simulation.
  void reset(const double t = 0) {
    t_ = t;
    qs_ = initial_angles_;
    vs_ = initial_vels_;
    as_ = JointValues();
    results_ = gtsam::Values();
  }

  /**
   * Perform forward dynamics to calculate accelerations, update a_, add new
   * values to results_
   * @param torques torques for the time step
   */
  void forwardDynamics(const JointValues &torques) {
    auto fk_results = robot_.forwardKinematics(qs_, vs_);
    gtsam::Values result =
        graph_builder_.linearSolveFD(robot_, t_, qs_, vs_, torques, fk_results);
    results_.insert(result);

    // update accelerations
    as_ = DynamicsGraph::jointAccelsMap(robot_, result, t_);
  }

  /**
   * Integrate to calculate new q, v for one time step, update q_, v_
   * @param torques torques for the time step
   * @param dt duration for the time step
   */
  void integration(const double dt) {
    JointValues vs_new, qs_new;
    for (JointSharedPtr joint : robot_.joints()) {
      std::string name = joint->name();
      vs_new[name] = vs_.at(name) + dt * as_.at(name);
      qs_new[name] = qs_.at(name) + dt * vs_.at(name) +
                     0.5 * as_.at(name) * std::pow(dt, 2);
    }
    vs_ = vs_new;
    qs_ = qs_new;
  }

  /**
   * Simulate for one time step, update q_, v_, a_, t_, add the new values into
   * result_.
   * @param torques torques for the
   * @param dt duration for the time step
   */
  void step(const JointValues &torques, const double dt) {
    forwardDynamics(torques);
    integration(dt);
    t_++;
  }

  /// Simulation for the specified sequence of torques.
  gtsam::Values simulate(const std::vector<JointValues> torques_seq,
                         const double dt) {
    for (const JointValues &torques : torques_seq) {
      step(torques, dt);
    }
    return results_;
  }

  /// Return joint angle values.
  const JointValues &getJointAngles() const { return qs_; }

  /// Return joint velocity values.
  const JointValues &getJointVelocities() const { return vs_; }

  /// Return joint acceleration values.
  const JointValues &getJointAccelerations() const { return as_; }

  /// Return all values during simulation.
  const gtsam::Values &getValues() const { return results_; }
};

}  // namespace gtdynamics
