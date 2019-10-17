/**
 * @file  Simulation.h
 * @brief robot arm simulation using forward dynamics factor graph
 * @Author: Mandy Xie
 */
#pragma once

#include <Arm.h>

namespace manipulator {
/**
 * Simulation is a class which simulate robot arm motion using forward
 * dynamics
 */
template <typename T>
class Simulation {
 private:
  double dt_;
  double dt_2_;
  Arm<T> robot_;
  int dof_;
  gtsam::Vector3 gravity_;

 protected:
  gtsam::Vector jointAngles, jointVelocities, jointAccelerations, jointTorques;

 public:
  /**
   * Constructor
   * Keyword arguments:
   *  time_step                -- simulation time step
   *  robot                    -- robotic manipulator
   *  initialJointAngles       -- initial joint angles
   *  initialJointVelocities   -- initial joint velocities
   */
  Simulation(double time_step, const Arm<T> &robot,
             const gtsam::Vector3 &gravity,
             const gtsam::Vector &initialJointAngles,
             const gtsam::Vector &initialJointVelocities)
      : dt_(time_step),
        dt_2_(time_step * time_step),
        robot_(robot),
        dof_(robot_.numLinks()+1),
        gravity_(gravity),
        jointAngles(initialJointAngles),
        jointVelocities(initialJointVelocities),
        jointAccelerations(gtsam::Vector::Zero(dof_)),
        jointTorques(gtsam::Vector::Zero(dof_)) {}
  ~Simulation() {}
  /// update joint angle values
  void updateJointAngles(const gtsam::Vector &newJointAngles) {
    jointAngles = newJointAngles;
  }
  /// update joint velocity values
  void updateJointVelocities(const gtsam::Vector &newJointVelocities) {
    jointVelocities = newJointVelocities;
  }

  /// update joint acceleration values
  void updateJointAccelerations(const gtsam::Vector &newJointAccelerations) {
    jointAccelerations = newJointAccelerations;
  }

  /// update joint torque values
  void updateJointTorques(const gtsam::Vector &known_torque);

  /// return joint angle values
  gtsam::Vector getJointAngles() { return jointAngles; }
  /// return joint velocity values
  gtsam::Vector getJointVelocities() { return jointVelocities; }

  /// return joint acceleration values
  gtsam::Vector getJointAccelerations() { return jointAccelerations; }

  /// return joint torque values
  gtsam::Vector getJointTorques() { return jointTorques; }

  /// calculate joint accelerations from forward dynamics factor graph
  gtsam::Vector accelerations(const gtsam::Vector &known_torque,
                              const gtsam::Vector &known_q,
                              const gtsam::Vector &known_qVel);

  /// integrate joint velocities and joint accelerations to get joint angles
  /// and joint velocities
  void integration(const gtsam::Vector &known_torque);
};

}  // namespace manipulator
