/**
 * @file  Simulation.cpp
 * @brief robot arm simulation using forward dynamics factor graph
 * @Author: Mandy Xie
 */
#include <Simulation.h>
#include <UrdfLink.h>

using namespace std;
using namespace gtsam;

namespace manipulator {
template <typename T>
void Simulation<T>::updateJointTorques(const gtsam::Vector &known_torque) {
  jointTorques = known_torque;
  for (int i = 0; i < dof_; ++i) {
    jointTorques[i] += robot_.link(i).springCoefficient() * jointAngles[i] +
                      robot_.link(i).dampingCoefficient() * jointVelocities[i];
  }
  jointTorques[dof_ - 1] =
      robot_.loopSpringCoefficient() * jointAngles[dof_ - 1] +
      robot_.loopDampingCoefficient() * jointVelocities[dof_ - 1];
}

template <typename T>
Vector Simulation<T>::accelerations(const gtsam::Vector &known_torque,
                                    const gtsam::Vector &known_q,
                                    const gtsam::Vector &known_qVel) {
  DynamicsFactorGraphInput<Vector> dynamicsInput(known_q, known_qVel, known_torque,
                                         gtsam::Vector6::Zero(),
                                         gtsam::Vector6::Zero());
  auto factor_graph =
      robot_.closedLoopForwardDynamicsFactorGraph(dynamicsInput, gravity_);
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

template class Simulation<UrdfLink>;
}  // namespace manipulator