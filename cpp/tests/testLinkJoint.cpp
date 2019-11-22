/**
 * @file  testLinkBody.cpp
 * @brief test LinkBody class
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include <LinkJoint.h>
#include <LinkBody.h>
#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace robot;
using namespace gtsam;

/**
 * construct a LinkJoint and ensure all values are as expected.
 */
TEST(LinkJoint, constructor) {
    std::string simple_urdf_str = manipulator::load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
    auto simple_urdf = manipulator::get_urdf(simple_urdf_str);

    LinkBodySharedPtr l1 = std::make_shared<LinkBody>(LinkBody(simple_urdf->links_["l1"]));
    LinkBodySharedPtr l2 = std::make_shared<LinkBody>(LinkBody(simple_urdf->links_["l2"]));
    LinkBodyWeakPtr l2_weak = l2->getWeakPtr();

    robot::LinkJointParams j1_params;
    j1_params.name = "j1";
    j1_params.jointEffortType = robot::LinkJoint::JointEffortType::Actuated;

    // Test constructor.
    LinkJointSharedPtr link_joint_strong = std::make_shared<LinkJoint>(
      LinkJoint(
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
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}