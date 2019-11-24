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

// Constructs LinkBody and LinkJoint objects from an input urdf:ModelInterface
// pointer with a loop and checks that constructed values have correct parents,
// children, and transforms.
TEST(UniversalRobot, test_extract_structure_with_loop_from_urdf) {
  // Obtain urdf::ModelInterfaceSharedPtr from sample urdf file.
  std::string four_bar_urdf_str = manipulator::load_file_into_string(
    "../../../urdfs/test/four_bar_linkage.urdf");
  auto four_bar_urdf = manipulator::get_urdf(four_bar_urdf_str);

  // Obtain LinkBody and JointBody objects from ModelInterfaceSharedPtr.
  LinkBodyJointPair urdf_bodies_and_joints = extract_structure_from_urdf(four_bar_urdf);
  std::vector<robot::LinkBodySharedPtr> linkBodies = urdf_bodies_and_joints.first;
  std::vector<robot::LinkJointSharedPtr> linkJoints = urdf_bodies_and_joints.second;

  EXPECT(assert_equal(5, linkBodies.size()));
  EXPECT(assert_equal(5, linkJoints.size()));

  // Grab all links.
  auto l0 = *std::find_if(linkBodies.begin(), linkBodies.end(), 
    [=] (const robot::LinkBodySharedPtr & link) { return (link->name() == "l0"); });
  auto l1 = *std::find_if(linkBodies.begin(), linkBodies.end(),
    [=] (const robot::LinkBodySharedPtr & link) { return (link->name() == "l1"); });
  auto l2 = *std::find_if(linkBodies.begin(), linkBodies.end(),
    [=] (const robot::LinkBodySharedPtr & link) { return (link->name() == "l2"); });
  auto l3 = *std::find_if(linkBodies.begin(), linkBodies.end(),
    [=] (const robot::LinkBodySharedPtr & link) { return (link->name() == "l3"); });
  auto l4 = *std::find_if(linkBodies.begin(), linkBodies.end(),
    [=] (const robot::LinkBodySharedPtr & link) { return (link->name() == "l4"); });

  // Check link l1's parents and children.
  EXPECT(assert_equal(2, l1->getParentLinks().size()));
  EXPECT(assert_equal(2, l1->getParentJoints().size()));
  EXPECT(assert_equal(1, l1->getChildLinks().size()));
  EXPECT(assert_equal(1, l1->getChildJoints().size()));

  // Check that l1's parent links are l0 and l4, its parent joints are j0 and j4, child
  // link is l2, and child joint is j1.
  EXPECT(assert_equal(
    1,
    ("l0" == l1->getParentLinks()[0]->name()) || ("l0" == l1->getParentLinks()[1]->name())
  ));
  EXPECT(assert_equal(
    1,
    ("l4" == l1->getParentLinks()[0]->name()) || ("l4" == l1->getParentLinks()[1]->name())
  ));
  EXPECT(assert_equal(
    1,
    "l2" == l1->getChildLinks()[0].lock()->name()
  ));
  EXPECT(assert_equal(
    1,
    ("j0" == l1->getParentJoints()[0]->name()) || ("j0" == l1->getParentJoints()[1]->name())
  ));
  EXPECT(assert_equal(
    1,
    ("j4" == l1->getParentJoints()[0]->name()) || ("j4" == l1->getParentJoints()[1]->name())
  ));
  EXPECT(assert_equal(
    1,
    "j1" == l1->getChildJoints()[0].lock()->name()
  ));


  // Check link l2's parents and children.
  EXPECT(assert_equal(1, l2->getParentLinks().size()));
  EXPECT(assert_equal(1, l2->getParentJoints().size()));
  EXPECT(assert_equal(1, l2->getChildLinks().size()));
  EXPECT(assert_equal(1, l2->getChildJoints().size()));

  // Check that l2's parent link is l1, its parent joint is j1, child
  // link is l3, and child joint is j2.
  EXPECT(assert_equal(
    1,
    "l1" == l2->getParentLinks()[0]->name()
  ));
  
  EXPECT(assert_equal(
    1,
    "l3" == l2->getChildLinks()[0].lock()->name()
  ));
  EXPECT(assert_equal(
    1,
    "j1" == l2->getParentJoints()[0]->name()
  ));
  EXPECT(assert_equal(
    1,
    "j2" == l2->getChildJoints()[0].lock()->name()
  ));

  // Check link l3's parents and children.
  EXPECT(assert_equal(1, l3->getParentLinks().size()));
  EXPECT(assert_equal(1, l3->getParentJoints().size()));
  EXPECT(assert_equal(1, l3->getChildLinks().size()));
  EXPECT(assert_equal(1, l3->getChildJoints().size()));

  // Check that l3's parent link is l2, its parent joint is j2, child
  // link is l4, and child joint is j3.
  EXPECT(assert_equal(
    1,
    "l2" == l3->getParentLinks()[0]->name()
  ));
  
  EXPECT(assert_equal(
    1,
    "l4" == l3->getChildLinks()[0].lock()->name()
  ));
  EXPECT(assert_equal(
    1,
    "j2" == l3->getParentJoints()[0]->name()
  ));
  EXPECT(assert_equal(
    1,
    "j3" == l3->getChildJoints()[0].lock()->name()
  ));

  // Check link l4's parents and children.
  EXPECT(assert_equal(1, l4->getParentLinks().size()));
  EXPECT(assert_equal(1, l4->getParentJoints().size()));
  EXPECT(assert_equal(1, l4->getChildLinks().size()));
  EXPECT(assert_equal(1, l4->getChildJoints().size()));

  // Check that l4's parent link is l3, its parent joint is j3, child
  // link is l1, and child joint is j4.
  EXPECT(assert_equal(
    1,
    "l3" == l4->getParentLinks()[0]->name()
  ));
  
  EXPECT(assert_equal(
    1,
    "l1" == l4->getChildLinks()[0].lock()->name()
  ));
  EXPECT(assert_equal(
    1,
    "j3" == l4->getParentJoints()[0]->name()
  ));
  EXPECT(assert_equal(
    1,
    "j4" == l4->getChildJoints()[0].lock()->name()
  ));
}

// Initialize a UniversalRobot with "urdfs/test/simple_urdf.urdf" and make sure
// that all transforms, link/joint properties, etc. are correct.
TEST(UniversalRobot, instantiate_from_urdf) {
    // Load urdf file into urdf::ModelInterfacePtr
    std::string simple_urdf_str = manipulator::load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
    auto simple_urdf = manipulator::get_urdf(simple_urdf_str);

    LinkBodyJointPair urdf_bodies_and_joints = extract_structure_from_urdf(simple_urdf);

    // Initialize UniversalRobot instance using LinkBody and LinkJoint instances.
    UniversalRobot simple_robot = UniversalRobot(urdf_bodies_and_joints, "l1", Pose3());

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}