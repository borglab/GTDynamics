/**
 * @file  testSphereLink.cpp
 * @brief test sphere link model used for collision check
 * @Author: Mandy Xie
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/VectorValues.h>

#include <SphereLink.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

TEST(SphereLink, test) {
  double radius = 0.1;
  vector<Point3> sphere_centers;
  for (int i = 0; i < 10; ++i) {
    sphere_centers.push_back(Point3((2 * i - 9) * radius, 0, 0));
  }
  SphereLink sphere_link(radius, sphere_centers);
  Pose3 com_pose = Pose3(Rot3(), Point3(1, 0, 0));

  Matrix actual_H, expected_H;
  Point3 actual_center, expected_center;
  // test for sphere index 0
  actual_center = sphere_link.sphereCenter(0, com_pose, actual_H);
  expected_center = Point3(0.1, 0, 0);
  EXPECT(assert_equal(expected_center, actual_center, 1e-6));
  expected_H = numericalDerivative11(
      boost::function<Point3(const Pose3&)>(boost::bind(
          &SphereLink::sphereCenter, sphere_link, 0, _1, boost::none)),
      com_pose, 1e-6);
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));

  // test for sphere index 4
  actual_center = sphere_link.sphereCenter(4, com_pose, actual_H);
  expected_center = Point3(0.9, 0, 0);
  EXPECT(assert_equal(expected_center, actual_center, 1e-6));
  expected_H = numericalDerivative11(
      boost::function<Point3(const Pose3&)>(boost::bind(
          &SphereLink::sphereCenter, sphere_link, 4, _1, boost::none)),
      com_pose, 1e-6);
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}