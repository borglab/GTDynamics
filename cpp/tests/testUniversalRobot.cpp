/**
 * @file testUniversalRobot.cpp
 * @brief test UniversalRobot instance methods and integration test with various URDF configurations
 * @Author Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <CppUnitLite/Test.h>
#include <UniversalRobot.h>
#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace robot;
using namespace gtsam;

// Constructs RobotLink and RobotJoint objects from an input urdf:ModelInterface
// pointer and checks that constructed values have correct parents, children,
// and transforms.
TEST(UniversalRobot, test_extract_structure_from_urdf) {
  // Obtain urdf::ModelInterfaceSharedPtr from sample urdf file.
  auto simple_urdf = get_sdf("../../../urdfs/test/simple_urdf.urdf");

  // Obtain RobotLink and JointBody objects from ModelInterfaceSharedPtr.
  std::vector<robot::RobotJointParams> joint_params;
  robot::RobotJointParams j1_params;
  j1_params.name = "j1";
  j1_params.jointEffortType = robot::RobotJoint::JointEffortType::Actuated;
  joint_params.push_back(j1_params);

  RobotJointPair urdf_bodies_and_joints = extract_structure_from_sdf(simple_urdf, 
    boost::optional<std::vector<robot::RobotJointParams>>(joint_params));
  std::vector<robot::RobotLinkSharedPtr> linkBodies = urdf_bodies_and_joints.first;
  std::vector<robot::RobotJointSharedPtr> RobotJoints = urdf_bodies_and_joints.second;

  EXPECT(assert_equal(2, linkBodies.size()));
  EXPECT(assert_equal(1, RobotJoints.size()));

  // Ensure that link l1 has link l2 listed as a child link and j1 listed as
  // a child joint. Ensure that link l2 has link l1 lsited as a parent link
  // and j1 listed as a parent joint.
  auto l1_it = std::find_if(linkBodies.begin(), linkBodies.end(), [=] (const robot::RobotLinkSharedPtr & link) {
    return (link->name() == "l1");
  });
  auto l2_it = std::find_if(linkBodies.begin(), linkBodies.end(), [=] (const robot::RobotLinkSharedPtr & link) {
    return (link->name() == "l2");
  });

  EXPECT(assert_equal(0, l1_it == linkBodies.end()));
  EXPECT(assert_equal(0, (*l1_it)->getParentLinks().size()));
  EXPECT(assert_equal(0, (*l1_it)->getParentJoints().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildLinks().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildJoints().size()));
  EXPECT(assert_equal(1, (*l1_it)->getJoints().size()));

  EXPECT(assert_equal(0, l2_it == linkBodies.end()));
  EXPECT(assert_equal(1, (*l2_it)->getParentLinks().size()));
  EXPECT(assert_equal(1, (*l2_it)->getParentJoints().size()));
  EXPECT(assert_equal(1, (*l1_it)->getChildLinks().size()));
  EXPECT(assert_equal(0, (*l2_it)->getChildJoints().size()));
  EXPECT(assert_equal(1, (*l2_it)->getJoints().size()));

  robot::RobotLinkWeakPtr l2_weak = (*l1_it)->getChildLinks()[0];
  EXPECT(assert_equal("l2", l2_weak.lock()->name()));
  EXPECT(assert_equal("l1", ((*l2_it)->getParentLinks()[0])->name()));

  auto j1_it = std::find_if(RobotJoints.begin(), RobotJoints.end(), [=] (const robot::RobotJointSharedPtr & joint) {
    return (joint->name() == "j1");
  });
  EXPECT(assert_equal(0, j1_it == RobotJoints.end()));
  EXPECT(assert_equal("j1", (*j1_it)->name()));
  EXPECT(assert_equal(0, j1_params.jointEffortType != (*j1_it)->jointEffortType()));
}

// Initialize a UniversalRobot with "urdfs/test/simple_urdf.urdf" and make sure
// that all transforms, link/joint properties, etc. are correct.
TEST(UniversalRobot, instantiate_from_urdf) {
    // Load urdf file into sdf::Model
    auto simple_urdf = get_sdf("../../../urdfs/test/simple_urdf.urdf");

    RobotJointPair urdf_bodies_and_joints = extract_structure_from_sdf(simple_urdf);

    // Initialize UniversalRobot instance using RobotLink and RobotJoint instances.
    UniversalRobot simple_robot = UniversalRobot(urdf_bodies_and_joints);

    // Check that number of links and joints in the UniversalRobot instance is correct.
    EXPECT(assert_equal(2, simple_robot.links().size()));
    EXPECT(assert_equal(1, simple_robot.joints().size()));

    EXPECT(assert_equal(
      "l1",
      simple_robot.getLinkByName("l1")->name()
    ));
    EXPECT(assert_equal(
      "l2",
      simple_robot.getLinkByName("l2")->name()
    ));
    EXPECT(assert_equal(
      "j1",
      simple_robot.getJointByName("j1")->name()
    ));

    EXPECT(assert_equal(
      -1.57, simple_robot.getJointByName("j1")->jointLowerLimit()));
    EXPECT(assert_equal(
      1.57, simple_robot.getJointByName("j1")->jointUpperLimit()));
    EXPECT(assert_equal(
      0.0, simple_robot.getJointByName("j1")->jointLimitThreshold()));

    EXPECT(assert_equal(
      Pose3(Rot3(), Point3(0, 0, -2)),
      simple_robot.joints()[0]->McpCom()
    ));

    EXPECT(assert_equal(
      Pose3(Rot3(), Point3(0, 0, 2)),
      simple_robot.joints()[0]->MpcCom()
    ));
}

TEST(UniversalRobot, instantiate_from_simple_urdf_file) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot simple = UniversalRobot("../../../urdfs/test/simple_urdf.urdf");

  // Check that number of links and joints in the UniversalRobot instance is correct.
  EXPECT(assert_equal(2, simple.links().size()));
  EXPECT(assert_equal(1, simple.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("l1", simple.getLinkByName("l1")->name()));
  EXPECT(assert_equal("l2", simple.getLinkByName("l2")->name()));
  EXPECT(assert_equal("j1", simple.getJointByName("j1")->name()));

  EXPECT(assert_equal(
    -1.57, simple.getJointByName("j1")->jointLowerLimit()));
  EXPECT(assert_equal(
    1.57, simple.getJointByName("j1")->jointUpperLimit()));
  EXPECT(assert_equal(
    0.0, simple.getJointByName("j1")->jointLimitThreshold()));
}

TEST(UniversalRobot, instantiate_from_urdf_file) {

  // Initialize UniversalRobot instance from a file.
  UniversalRobot four_bar = UniversalRobot("../../../sdfs/test/four_bar_linkage.sdf");

  // Check that number of links and joints in the UniversalRobot instance is correct.
  EXPECT(assert_equal(5, four_bar.links().size()));
  EXPECT(assert_equal(5, four_bar.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("l0", four_bar.getLinkByName("l0")->name()));
  EXPECT(assert_equal("l1", four_bar.getLinkByName("l1")->name()));
  EXPECT(assert_equal("l2", four_bar.getLinkByName("l2")->name()));
  EXPECT(assert_equal("l3", four_bar.getLinkByName("l3")->name()));
  EXPECT(assert_equal("l4", four_bar.getLinkByName("l4")->name()));
  EXPECT(assert_equal("j0", four_bar.getJointByName("j0")->name()));
  EXPECT(assert_equal("j1", four_bar.getJointByName("j1")->name()));
  EXPECT(assert_equal("j2", four_bar.getJointByName("j2")->name()));
  EXPECT(assert_equal("j3", four_bar.getJointByName("j3")->name()));
  EXPECT(assert_equal("j4", four_bar.getJointByName("j4")->name()));

  EXPECT(assert_equal(
    -1.57, four_bar.getJointByName("j1")->jointLowerLimit()));
  EXPECT(assert_equal(
    1.57, four_bar.getJointByName("j1")->jointUpperLimit()));
  EXPECT(assert_equal(
    0.0, four_bar.getJointByName("j1")->jointLimitThreshold()));  
}

TEST(UniversalRobot, instantiate_from_sdf_file) {

  // Initialize UniversalRobot instance from a file.
  UniversalRobot simple_rr = UniversalRobot("../../../sdfs/test/simple_rr.sdf", "simple_rr_sdf");

  // // Check that number of links and joints in the UniversalRobot instance is correct.
  EXPECT(assert_equal(3, simple_rr.links().size()));
  EXPECT(assert_equal(2, simple_rr.joints().size()));

  // Test getLinkByName(...) and getJointByName(...)
  EXPECT(assert_equal("link_0", simple_rr.getLinkByName("link_0")->name()));
  EXPECT(assert_equal("link_1", simple_rr.getLinkByName("link_1")->name()));
  EXPECT(assert_equal("link_2", simple_rr.getLinkByName("link_2")->name()));

  EXPECT(assert_equal("joint_1", simple_rr.getJointByName("joint_1")->name()));
  EXPECT(assert_equal("joint_2", simple_rr.getJointByName("joint_2")->name()));

  EXPECT(assert_equal(
    -1e16, simple_rr.getJointByName("joint_1")->jointLowerLimit()));
  
  EXPECT(assert_equal(
    1e16, simple_rr.getJointByName("joint_1")->jointUpperLimit()));
  
  EXPECT(assert_equal(
    0.0, simple_rr.getJointByName("joint_1")->jointLimitThreshold()));

}

TEST(UniversalRobot, removeLink) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot four_bar = UniversalRobot("../../../sdfs/test/four_bar_linkage_pure.sdf");
  four_bar.removeLink(four_bar.getLinkByName("l2"));
  EXPECT(four_bar.numLinks() == 3);
  EXPECT(four_bar.numJoints() == 2);
  EXPECT(four_bar.getLinkByName("l1")->getJoints().size() == 1);
  EXPECT(four_bar.getLinkByName("l3")->getJoints().size() == 1);
}

TEST(UniversalRobot, jumping_robot) {
  // Initialize UniversalRobot instance from a file.
  UniversalRobot jumping_robot = UniversalRobot("../../../urdfs/test/jumping_robot.urdf");
  // jumping_robot.getLinkByName("l0")->fix();
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}