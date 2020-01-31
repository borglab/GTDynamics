/**
 * @file  testRobotLink.cpp
 * @brief test RobotLink class
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

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
 * 
 * construct a RobotLink via urdf link and ensure all values are as expected.
 */
TEST(RobotLink, urdf_constructor) {
    auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

    // Initialize UniversalRobot instance using urdf::ModelInterfacePtr.
    RobotLink first_link = RobotLink(*simple_urdf.LinkByName("l1"));

    EXPECT(assert_equal("l1", first_link.name()));
    EXPECT(assert_equal(100, first_link.mass()));

    // Check center of mass.
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), first_link.Tlcom()));

    // Check inertia.
    EXPECT(assert_equal(
        (gtsam::Matrix(3,3) << 3, 0, 0, 0, 2, 0, 0, 0, 1).finished(),
        first_link.inertia()
    ));

    // Check general mass matrix.
    EXPECT(assert_equal(
        (gtsam::Matrix(6,6) << 3, 0, 0, 0, 0, 0,
                               0, 2, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 100, 0, 0,
                               0, 0, 0, 0, 100, 0,
                               0, 0, 0, 0, 0, 100).finished(),
        first_link.inertiaMatrix()
    ));

    // Check that no child links/joints have yet been added.
    EXPECT(assert_equal(0, first_link.getChildLinks().size()));
    EXPECT(assert_equal(0, first_link.getChildJoints().size()));

    // Add child link.
    RobotLink second_link = RobotLink(*simple_urdf.LinkByName("l2"));
    
    second_link.addParentLink(std::make_shared<RobotLink>(first_link));

    // Check that the second link's parent is link 1.
    EXPECT(assert_equal(first_link.name(), second_link.getParentLinks()[0]->name()));

    first_link.addChildLink(std::make_shared<RobotLink>(second_link));

    // Assert correct center of mass in link frame.
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), first_link.Tlcom()));

    // Check transform to link-end frame from link com frame. leTl_com
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, -1)), first_link.leTl_com()));

    // Test that ID is set correctly.
    unsigned char id = 'a';
    first_link.setID(id);
    EXPECT(assert_equal((double) first_link.getID(), (double) id));
}

TEST(RobotLink, sdf_constructor) {
    auto model = get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

    RobotLink l0 = RobotLink(*model.LinkByName("link_0"));
    RobotLink l1 = RobotLink(*model.LinkByName("link_1"));

    // Both link frames are defined in the world frame.
    EXPECT(assert_equal(gtsam::Pose3::identity(), l0.Twl()));
    EXPECT(assert_equal(gtsam::Pose3::identity(), l1.Twl()));

    // Verify center of mass defined in the link frame is correct.
    EXPECT(assert_equal(
        gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1)),
        l0.Tlcom()));
    EXPECT(assert_equal(
        gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.5)),
        l1.Tlcom()));

    // Verify center of mass defined in the world frame is correct.
    EXPECT(assert_equal(
        gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.1)),
        l0.Twcom()));
    EXPECT(assert_equal(
        gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0.5)),
        l1.Twcom()));

    // Verify that mass is correct.
    EXPECT(assert_equal(0.01, l0.mass()));
    EXPECT(assert_equal(0.01, l1.mass()));

    // Verify that inertia elements are correct.
    EXPECT(assert_equal(
        (gtsam::Matrix(3, 3) << 0.05, 0, 0, 0, 0.06, 0, 0, 0, 0.03).finished(),
        l0.inertia()));
    EXPECT(assert_equal(
        (gtsam::Matrix(3, 3) << 0.05, 0, 0, 0, 0.06, 0, 0, 0, 0.03).finished(),
        l1.inertia()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
