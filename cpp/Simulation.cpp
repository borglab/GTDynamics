/**
 * @file  Simulation.cpp
 * @brief robot arm simulation using forward dynamics factor graph
 * @Author: Mandy Xie
 */
#include <Simulation.h>
#include <URDFLink.h>

using namespace std;
using namespace gtsam;

namespace manipulator {
template <typename T>
void Simulation<T>::updateJointTorques(const gtsam::Vector &known_torque) {
  jointTorques = known_torque;
  for(int i = 0; i < dof_; ++i) {
    if (robot_.link(i).jointEffortType() == Link::Impedence) {
      jointTorques[i] = -1500 * jointAngles[i];
    }
    if (robot_.loopJointEffortType() == Link::Impedence) {
      jointTorques[dof_-1] = -1500 * jointAngles[dof_-1];
    }
  }
}

template <typename T>
Vector Simulation<T>::accelerations(const gtsam::Vector &known_torque,
                                    const gtsam::Vector &known_q,
                                    const gtsam::Vector &known_qVel) {
  auto factor_graph = robot_.closedLoopForwardDynamicsFactorGraph(
      known_q, known_qVel, known_torque, gtsam::Vector6::Zero(),
      gtsam::Vector6::Zero(), gravity_);
  VectorValues results = factor_graph.optimize();
  return robot_.extractJointAcceleraions(results, dof_);
}

template <typename T>
void Simulation<T>::integration(const gtsam::Vector &known_torque) {
  auto newJointAccelerations = accelerations(known_torque, jointAngles, jointVelocities);
  auto newJointVelocities = jointVelocities + newJointAccelerations * dt_;
  auto newJointAngles =
      jointAngles + jointVelocities * dt_ + 0.5 * newJointAccelerations * dt_2_;
  updateJointAccelerations(newJointAccelerations);
  updateJointVelocities(newJointVelocities);
  updateJointAngles(newJointAngles);
  updateJointTorques(known_torque);
}

template <typename T>
void Simulation<T>::rungeKutta4(const gtsam::Vector &known_torque) {
  double c = 1.0 / 6, c1 = 1.0 / 3, c2 = 1.0 / 3, c3 = 1.0 / 6;
  Vector q1, q2, q3, q4;
  Vector qVel1, qVel2, qVel3, qVel4;
  Vector qAccel1, qAccel2, qAccel3, qAccel;
  qAccel = accelerations(known_torque, jointAngles, jointVelocities);
  qVel1 = jointVelocities + 0.5 * qAccel * dt_;
  q1 = jointAngles + 0.5 * jointVelocities * dt_;
  qAccel1 = accelerations(known_torque, q1, qVel1);
  qVel2 = jointVelocities + 0.5 * qAccel1 * dt_;
  q2 = jointAngles + 0.5 * qVel1 * dt_;
  qAccel2 = accelerations(known_torque, q2, qVel2);
  qVel3 = jointVelocities + qAccel2 * dt_;
  q3 = jointAngles + qVel2 * dt_;
  qAccel3 = accelerations(known_torque, q3, qVel3);
  qVel4 = jointVelocities +
          dt_ * (c * qAccel + c1 * qAccel1 + c2 * qAccel2 + c3 * qAccel3);
  q4 = jointAngles +
       dt_ * (c * jointVelocities + c1 * qVel1 + c2 * qVel2 + c3 * qVel3);
  updateJointAccelerations(qAccel3);
  updateJointVelocities(qVel4);
  updateJointAngles(q4);
  updateJointTorques(known_torque);
}
template class Simulation<URDF_Link>;
}  // namespace manipulator