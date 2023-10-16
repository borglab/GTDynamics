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

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <optional>
#include <string>
#include <vector>

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
  gtsam::Values initial_values_;
  std::optional<gtsam::Vector3> gravity_;
  std::optional<gtsam::Vector3> planar_axis_;
  gtsam::Values current_values_;
  gtsam::Values new_kinematics_;

 public:
  /**
   * Constructor
   *
   * @param time_step      Simulator time step
   * @param robot          robotic robot
   * @param initial_values initial joint angles and velocities
   * @param gravity        gravity vector
   * @param planar_axis    planar axis vector
   */
  Simulator(const Robot &robot, const gtsam::Values &initial_values,
            const std::optional<gtsam::Vector3> &gravity = {},
            const std::optional<gtsam::Vector3> &planar_axis = {})
      : robot_(robot),
        t_(0),
        graph_builder_(DynamicsGraph(gravity, planar_axis)),
        initial_values_(initial_values) {
    reset();
  }
  ~Simulator() {}

  /// Reset simulation.
  void reset(const double t = 0) {
    t_ = t;
    new_kinematics_ = initial_values_;
  }

  /**
   * Perform forward dynamics to calculate accelerations.
   * @param torques torques for the time step
   */
  void forwardDynamics(const gtsam::Values &torques) {
    // Do FK to add poses
    auto values = robot_.forwardKinematics(new_kinematics_);

    // Add torques
    for (auto &&joint : robot_.joints()) {
      auto j = joint->id();
      InsertTorque(&values, j, Torque(torques, j));
    }

    // Now compute accelerations with forward dynamics
    current_values_ = graph_builder_.linearSolveFD(robot_, 0, values);
  }

  /**
   * Integrate to calculate new q, v for one time step, update q_, v_
   * @param torques torques for the time step
   * @param dt duration for the time step
   */
  void integration(const double dt) {
    new_kinematics_ = gtsam::Values();
    const double dt2 = std::pow(dt, 2);
    for (auto &&joint : robot_.joints()) {
      auto j = joint->id();
      const double q = JointAngle(current_values_, j);
      const double v = JointVel(current_values_, j);
      const double a = JointAccel(current_values_, j);

      // TODO(frank): one could use t values and save entire simulation.
      const double v_new = v + dt * a;
      InsertJointVel(&new_kinematics_, j, v_new);
      // TODO(frank): consider using v_new for symplectic integration.
      InsertJointAngle(&new_kinematics_, j, q + dt * v + 0.5 * a * dt2);
    }
  }

  /**
   * Simulate for one time step.
   * @param torques torques for the
   * @param dt duration for the time step
   */
  void step(const gtsam::Values &torques, const double dt) {
    forwardDynamics(torques);
    integration(dt);
    t_++;
  }

  /// Simulation for the specified sequence of torques.
  gtsam::Values simulate(const std::vector<gtsam::Values> &torques_seq,
                         const double dt) {
    for (const auto &torques : torques_seq) {
      step(torques, dt);
    }
    return current_values_;
  }

  /// Return all values during simulation.
  const gtsam::Values &getValues() const { return current_values_; }
};

}  // namespace gtdynamics
