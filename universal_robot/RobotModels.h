#include <DynamicsGraph.h>
#include <UniversalRobot.h>

using namespace std;
using namespace robot;
using namespace gtsam;

// namespace jumping_robot
// {
// UniversalRobot getJumpingRobot()
// {
//   std::cout << "constructing" << std::endl;
//   UniversalRobot jumping_robot = UniversalRobot("../../../sdfs/test/jumping_robot.sdf");
//   std::cout << "constructed" << std::endl;
//   // jumping_robot.getLinkByName("l0")->fix();
//   return jumping_robot;
// }
// // Load the robot from urdf file
// UniversalRobot my_robot = getJumpingRobot();
// Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
// Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();
// Vector joint_angles = Vector::Zero(my_robot.numJoints());
// Vector joint_vels = Vector::Zero(my_robot.numJoints());
// } // namespace jumping_robot

namespace four_bar_linkage
{
UniversalRobot getFourBar()
{
  UniversalRobot four_bar = UniversalRobot(std::string(SDF_PATH) + "/test/four_bar_linkage_pure.sdf");
  return four_bar;
}
// Load the robot from urdf file
UniversalRobot my_robot = getFourBar();
Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();
Vector joint_angles = Vector::Zero(my_robot.numJoints());
Vector joint_vels = Vector::Zero(my_robot.numJoints());
} // namespace four_bar_linkage


namespace simple_urdf
{
UniversalRobot getSimpleUrdf()
{
  UniversalRobot simple_robot = UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");
  simple_robot.getLinkByName("l1")->fix();
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdf();
Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();
Vector joint_angles = Vector::Zero(my_robot.numJoints());
Vector joint_vels = Vector::Zero(my_robot.numJoints());
} // namespace simple_urdf

namespace simple_urdf_zero_inertia
{
UniversalRobot getSimpleUrdf()
{
  UniversalRobot simple_robot = UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf_zero_inertia.urdf");
  simple_robot.getLinkByName("l1")->fix();
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdf();
Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();
Vector joint_angles = Vector::Zero(my_robot.numJoints());
Vector joint_vels = Vector::Zero(my_robot.numJoints());
} // namespace simple_urdf_zero_inertia

namespace simple_urdf_eq_mass
{
UniversalRobot getSimpleUrdfEqMass()
{
  UniversalRobot simple_robot = UniversalRobot(std::string(URDF_PATH) + "/test/simple_urdf_eq_mass.urdf");
  return simple_robot;
}
UniversalRobot my_robot = getSimpleUrdfEqMass();
Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();
Vector joint_angles = Vector::Zero(my_robot.numJoints());
Vector joint_vels = Vector::Zero(my_robot.numJoints());
} // namespace simple_urdf_eq_mass