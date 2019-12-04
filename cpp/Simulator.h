/**
 * @file  Simulator.h
 * @brief robot Simulator using forward dynamics factor graph
 * @Author:Yetong Zhang
 */
#pragma once

#include <UniversalRobot.h>
#include <DynamicsGraph.h>

namespace robot {
/**
 * Simulator is a class which simulate robot arm motion using forward
 * dynamics
 */
template <typename T>
class Simulator {
 private:
  UniversalRobot robot_;
  gtsam::Vector3 gravity_;
  gtsam::Vector qs_, vs_, as_, torques_;

 public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- Simulator time step
   *  robot                    -- robotic manipulator
   *  initialJointAngles       -- initial joint angles
   *  initialJointVelocities   -- initial joint velocities
   */
  Simulator(const UniversalRobot &robot,
             const gtsam::Vector3 &gravity,
             const gtsam::Vector &initialJointAngles,
             const gtsam::Vector &initialJointVelocities)
      : robot_(robot),
        gravity_(gravity),
        qs_(initialJointAngles),
        vs_(initialJointVelocities),
        as_(gtsam::Vector::Zero(robot.numJoints())),
        torques_(gtsam::Vector::Zero(robot.numJoints())) {}
  ~Simulator() {}

  // reset the configuraiton of robot
  void resetRobot(const UniversalRobot &robot) {
  }

  // simulation for one step with given torques
  void step(const gtsam::Vector &known_torques) {
  }

  /// return joint angle values
  gtsam::Vector getJointAngles() { return qs_; }

  /// return joint velocity values
  gtsam::Vector getJointVelocities() { return vs_; }

  /// return joint acceleration values
  gtsam::Vector getJointAccelerations() { return as_; }

  /// return joint torque values
  gtsam::Vector getJointTorques() { return torques_; }
};

}  // namespace manipulator
