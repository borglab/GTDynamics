/**
 * @file testSimulation.cpp
 * @brief test simulation of robotic manipulator with closed kinematic loop
 * @Author: Mandy Xie
 */

#include <Simulation.h>
#include <UrdfLink.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const double HALF_PI = M_PI / 2;

namespace example {
vector<UrdfLink> urdf_rrr = {
    UrdfLink(Pose3(Rot3::Rz(HALF_PI), Point3(-1, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, true, 0, 0),
    UrdfLink(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, false, 0, 0),
    UrdfLink(Pose3(Rot3::Rz(-HALF_PI), Point3(2, 0, 0)), Vector3(0, 0, 1), 'R', 
              1, Pose3(Rot3(), Point3(1, 0, 0)), Z_3x3, false, 0, 0)};
Pose3 base = Pose3();
Pose3 tool = Pose3(Rot3(), Point3(2, 0, 0));

// get screw_axis for loop closure
auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(1, 0, 0));
auto robot = Arm<UrdfLink>(urdf_rrr, base, tool, screw_axis, false, 0, 0);
auto dof = robot.numLinks() + 1;
// required joint acceleration and applied torque at Inverse Dynamics
Vector qAccel_ID = Vector::Zero(dof);
Vector torque_ID = Vector::Zero(dof);
}  // namespace example

/* ************************************************************************/
/**
 * Test simulation with gravity in x direction, zero joint angles
 */
TEST(Simulation, gravity_x) {
  double time_step = 0.001;
  int total_steps = 5000;
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  Vector3 gravity = (Vector(3) << 9.8, 0, 0).finished();
  Simulation<UrdfLink> FDsim(time_step, example::robot, gravity,
                              initialJointAngles, initialJointVelocities);
  vector<Vector> jointAngles, jointVelocities, jointAccelerations, jointTorques;
  jointAngles.assign(total_steps, Vector::Zero(example::dof));
  jointVelocities.assign(total_steps, Vector::Zero(example::dof));
  jointAccelerations.assign(total_steps, Vector::Zero(example::dof));
  jointTorques.assign(total_steps, Vector::Zero(example::dof));
  ofstream q, qVel, qAccel, qTorque;
  q.open("../../../matlab/dataset/joint_angles/q.txt");
  qVel.open("../../../matlab/dataset/joint_angles/qVel.txt");
  qAccel.open("../../../matlab/dataset/joint_angles/qAccel.txt");
  qTorque.open("../../../matlab/dataset/joint_angles/qTorque.txt");
  for (int i = 0; i < total_steps; ++i) {
    FDsim.integration(known_torque);
    jointAngles.push_back(FDsim.getJointAngles());
    jointVelocities.push_back(FDsim.getJointVelocities());
    jointAccelerations.push_back(FDsim.getJointAccelerations());
    jointTorques.push_back(FDsim.getJointTorques());
    q << std::setprecision(16) << FDsim.getJointAngles().transpose() << std::endl;
    qVel << std::setprecision(16) << FDsim.getJointVelocities().transpose() << std::endl;
    qAccel << std::setprecision(16) << FDsim.getJointAccelerations().transpose() << std::endl;
    qTorque << std::setprecision(16) << FDsim.getJointTorques().transpose() << std::endl;
  }
  q.close();
  qVel.close();
  qAccel.close();
  qTorque.close();
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
