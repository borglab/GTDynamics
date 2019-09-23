/**
 * @file testSimulation.cpp
 * @brief test simulation of robotic manipulator with closed kinematic loop
 * @Author: Mandy Xie
 */

#include <Simulation.h>
#include <URDFLink.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

static const double HALF_PI = M_PI / 2;

/* cassie_agility.urdf
    joint1: left-knee-joint (active joint)
            parent link: left-thigh (considered as base, cannot move)
            child link: left-knee
    joint2: left-shin-joint (passive joint, but can exert resistant torque
            because of spring)
            parent link: left-knee
            child link: left-shin
    joint3: left-tarsus-joint
            parent link: left-shin
            child link: left-tarsus
    joint4: left-heel-spring-joint
            parent link: left-tarsus
            child link: left-heel-spring
    joint5: left-rod-joint
            (passive joint, but can exert resistant torque because of spring)
            parent link: left-heel-spring
            child link: left-achilles-rod
    joint6: loop closure joint
            parent link: left-achilles-rod
            child link: left-thigh
*/
namespace example {
vector<URDF_Link> urdf_cassie = {
    URDF_Link(Pose3(Rot3(), Point3(0.12, 0, 0.0045)), Vector3(0, 0, 1),
        'R', 0.7578, Pose3(Rot3(), Point3(0.023, 0.03207, -0.002181)),
        (Matrix(3, 3) << 0.001376, -0.00039744, -4.085e-05, -0.00039744,
         0.0010335, -5.374e-05, -4.085e-05, -5.374e-05, 0.0021637)
            .finished(), true, 0, -1),
    URDF_Link(Pose3(Rot3(), Point3(0.06068, 0.04741, 0)),
        Vector3(0, 0, 1), 'R', 0.577,
        Pose3(Rot3(), Point3(0.18338, 0.001169, 0.0002123)),
        (Matrix(3, 3) << 0.00035939, -0.00020981, 2.266e-05, -0.00020981,
         0.014728, -1.2e-07, 2.266e-05, -1.2e-07, 0.014707)
            .finished(), false, -1500, -0.1),
    URDF_Link(Pose3(Rot3(), Point3(0.43476, 0.02, 0)), Vector3(0, 0, 1),
        'R', 0.782,
        Pose3(Rot3(), Point3(0.11046, -0.03058, -0.00131)),
        (Matrix(3, 3) << 0.00039238, 0.00023651, -4.987e-05, 0.00023651,
         0.013595, -4.82e-06, -4.987e-05, -4.82e-06, 0.013674)
            .finished(), false, 0, -0.1),
    URDF_Link(Pose3(Rot3::RzRyRx(0, 0, 2.7207), Point3(-0.01269, -0.03059, 0)),
        Vector3(0, 0, 1), 'R', 0.126,
        Pose3(Rot3(), Point3(0.081, 0.0022, 0)),
        (Matrix(3, 3) << 2.959e-05, 7.15e-06, -6e-07, 7.15e-06, 0.00022231,
         1e-07, -6e-07, 1e-07, 0.0002007)
            .finished(), false, -1250, 0),
    URDF_Link(Pose3(Rot3::RzRyRx(0, 0, 0.608212), Point3(0.11877, -0.01, 0)),
        Vector3(0, 0, 1), 'R', 0.157,
        Pose3(Rot3(), Point3(0.2472, 0, 0)),
        Vector3(0.000004, 0.014061, 0.014061).asDiagonal(), false, 0, 0)};

Pose3 base = Pose3(Rot3::Rz(-M_PI / 2), Point3(0, 0, 0));
Pose3 tool = Pose3(Rot3(), Point3(0.5012, 0, 0));
// get screw_axis for loop closure
auto screw_axis = unit_twist(Vector3(0, 0, 1), Vector3(0.2540, 0, 0)); //0.5012-0.2472
auto robot = Arm<URDF_Link>(urdf_cassie, base, tool, screw_axis, false, 0, 0);
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
  double time_step = 0.0005;
  int total_steps = 20000;
  Vector initialJointAngles = Vector::Zero(example::dof),
         initialJointVelocities = Vector::Zero(example::dof),
         known_torque = Vector::Zero(example::dof);
  initialJointAngles << -1.201826, 0, 1.428819, 0, -1.481429, -1.254439;
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();
  Simulation<URDF_Link> FDsim(time_step, example::robot, gravity,
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
    q << FDsim.getJointAngles().transpose() << std::endl;
    qVel << FDsim.getJointVelocities().transpose() << std::endl;
    qAccel << FDsim.getJointAccelerations().transpose() << std::endl;
    qTorque << FDsim.getJointTorques().transpose() << std::endl;
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
