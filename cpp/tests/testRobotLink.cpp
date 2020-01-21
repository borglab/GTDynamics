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
 * construct a RobotLink and ensure all values are as expected.
 */
TEST(RobotLink, constructor) {
    std::string simple_urdf_str = load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
    auto simple_urdf = get_urdf(simple_urdf_str);

    // Initialize UniversalRobot instance using urdf::ModelInterfacePtr.
    RobotLink first_link = RobotLink(simple_urdf->links_["l1"]);

    EXPECT(assert_equal("l1", first_link.name()));
    EXPECT(assert_equal(100, first_link.mass()));

    // Check center of mass.
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), first_link.centerOfMass()));

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
    RobotLink second_link = RobotLink(simple_urdf->links_["l2"]);
    
    second_link.addParentLink(std::make_shared<RobotLink>(first_link));

    // Check that the second link's parent is link 1.
    EXPECT(assert_equal(first_link.name(), second_link.getParentLinks()[0]->name()));

    first_link.addChildLink(std::make_shared<RobotLink>(second_link));

    // Check transform to link-end frame from link frame. leTl
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 2)), first_link.leTl()));
    // Check transform to link-end frame from link com frame. leTl_com
    EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)), first_link.leTl_com()));

    // Test that ID is set correctly.
    unsigned char id = 'a';
    first_link.setID(id);
    EXPECT(assert_equal((double) first_link.getID(), (double) id));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
