/**
 * @file  testRobotLink.cpp
 * @brief test RobotLink class
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <RobotJoint.h>
#include <RobotLink.h>
#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace robot;
using namespace gtsam;

/**
 * construct a RobotJoint and ensure all values are as expected.
 */
TEST(RobotJoint, constructor) {
    std::string simple_urdf_str = load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
    auto simple_urdf = get_urdf(simple_urdf_str);

    RobotLinkSharedPtr l1 = std::make_shared<RobotLink>(RobotLink(simple_urdf->links_["l1"]));
    RobotLinkSharedPtr l2 = std::make_shared<RobotLink>(RobotLink(simple_urdf->links_["l2"]));
    RobotLinkWeakPtr l2_weak = l2->getWeakPtr();

    robot::RobotJointParams j1_params;
    j1_params.name = "j1";
    j1_params.jointEffortType = robot::RobotJoint::JointEffortType::Actuated;

    // Test constructor.
    RobotJointSharedPtr link_joint_strong = std::make_shared<RobotJoint>(
      RobotJoint(
        simple_urdf->joints_["j1"], j1_params.jointEffortType, j1_params.springCoefficient,
        j1_params.jointLimitThreshold, j1_params.velocityLimitThreshold, j1_params.accelerationLimit,
        j1_params.accelerationLimitThreshold, j1_params.torqueLimitThreshold, l1,
        l2_weak));

    // Rest transform is equivalent to transform with q = 0.
    EXPECT(assert_equal(link_joint_strong->pMc(), link_joint_strong->pTc(0)));
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 2)), link_joint_strong->pMc()));

    // Test that parent to child link transform is correct for -pi/2 and pi/2.
    EXPECT(assert_equal(Pose3(Rot3::Rx(-M_PI / 2), Point3(0, 0, 2)), link_joint_strong->pTc(-M_PI / 2)));
    EXPECT(assert_equal(Pose3(Rot3::Rx(M_PI / 2), Point3(0, 0, 2)), link_joint_strong->pTc(M_PI / 2)));

    // Test that ID is set correctly.
    unsigned char id = 'a';
    link_joint_strong->setID(id);
    EXPECT(assert_equal((double) link_joint_strong->getID(), (double) id));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}