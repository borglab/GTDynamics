/**
 * @file testUniversalRobot.cpp
 * @brief test UniversalRobot instance methods and integration test with various URDF configurations
 * @Author Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <UniversalRobot.h>
#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace robot;
using namespace gtsam;

// Initialize a UniversalRobot with "urdfs/test/simple_urdf.urdf" and make sure
// that all transforms, link/joint properties, etc. are correct.
TEST(UniversalRobot, instantiate_from_urdf) {
    // Load urdf file into urdf::ModelInterfacePtr
    std::string simple_urdf_str = manipulator::load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
    auto simple_urdf = manipulator::get_urdf(simple_urdf_str);

    // Initialize UniversalRobot instance using urdf::ModelInterfacePtr.
    UniversalRobot simple_robot = UniversalRobot(simple_urdf, Pose3());

    // Check that number of links and joints in the UniversalRobot instance is correct.
    // EXPECT(assert_equal(2, simple_robot.numLinks()));
    // EXPECT(assert_equal(1, simple_robot.numJoints()));

    // This robot contains no loops.
    // EXPECT(assert_equal(0, simple_robot.numLoops()));

    // Check rest configuration transforms (q = 0).
    // check here.

    // Ensure that link properties are correct.
    // check all links here.

    // Ensure that joint properties are correct.
    // check all joints here.

    // Test jTi_list.

    // Test cTp_com_list.

    // Test screw axes.

    // Test spatial screw axes.

    // Test transform poe.

    // Test spatial jacobian.

    // Test body jacobian.

    // Test inverse kinematics.

    // Test length.
}

// Constructs LinkBody and LinkJoint objects from an input urdf:ModelInterface
// pointer and checks that constructed values have correct parents, children,
// and transforms.
TEST(UniversalRobot, test_extract_structure_from_urdf) {
  // Obtain urdf::ModelInterfaceSharedPtr from sample urdf file.
  std::string simple_urdf_str = manipulator::load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
  auto simple_urdf = manipulator::get_urdf(simple_urdf_str);

  // Obtain LinkBody and JointBody objects from ModelInterfaceSharedPtr.
  std::vector<robot::LinkJointParams> joint_params;
  robot::LinkJointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = robot::LinkJoint::JointEffortType::Actuated;
  joint_params.push_back(j1_params);

  LinkBodyJointPair urdf_bodies_and_joints = extract_structure_from_urdf(simple_urdf, 
    boost::optional<std::vector<robot::LinkJointParams>>(joint_params));
  std::vector<robot::LinkBodySharedPtr> linkBodies = urdf_bodies_and_joints.first;
  std::vector<robot::LinkJointSharedPtr> linkJoints = urdf_bodies_and_joints.second;

  EXPECT(assert_equal(2, linkBodies.size()));
  EXPECT(assert_equal(1, linkJoints.size()));

  // Ensure that link l1 has link l2 listed as a child link and j1 listed as
  // a child joint. Ensure that link l2 has link l1 lsited as a parent link
  // and j1 listed as a parent joint.
  auto l1_it = std::find_if(linkBodies.begin(), linkBodies.end(), [=] (const robot::LinkBodySharedPtr & link) {
    return (link->name() == "l1");
  });
  auto l2_it = std::find_if(linkBodies.begin(), linkBodies.end(), [=] (const robot::LinkBodySharedPtr & link) {
    return (link->name() == "l2");
  });

  EXPECT(assert_equal(0, l1_it == linkBodies.end()));
  EXPECT(assert_equal(0, (*l1_it)->getParentLinks().size()));
  EXPECT(assert_equal(0, (*l1_it)->getParentJoints().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildLinks().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildJoints().size()));

  EXPECT(assert_equal(0, l2_it == linkBodies.end()));
  EXPECT(assert_equal(1, (*l2_it)->getParentLinks().size()));
  EXPECT(assert_equal(1, (*l2_it)->getParentJoints().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildLinks().size()));
  EXPECT(assert_equal(0, (*l2_it)->getChildJoints().size()));

  robot::LinkBodyWeakPtr l2_weak = (*l1_it)->getChildLinks()[0];
  EXPECT(assert_equal("l2", l2_weak.lock()->name()));
  EXPECT(assert_equal("l1", ((*l2_it)->getParentLinks()[0])->name()));

  auto j1_it = std::find_if(linkJoints.begin(), linkJoints.end(), [=] (const robot::LinkJointSharedPtr & joint) {
    return (joint->name() == "j1");
  });
  EXPECT(assert_equal(0, j1_it == linkJoints.end()));
  EXPECT(assert_equal("j1", (*j1_it)->name()));
  EXPECT(assert_equal(0, j1_params.jointEffortType != (*j1_it)->jointEffortType()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}