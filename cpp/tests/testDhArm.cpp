/**
 * @file  testDhArm.cpp
 * @brief test robotic arm with DH links
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <DhArm.h>
#include <DhLink.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// Unit tests for dh link RR
TEST(DhArm, DH_RR) {
  vector<DhLink> dh_rr = {
      DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), I_3x3, -180, 180, 20),
      DhLink(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), I_3x3, -180, 180, 20)};

  auto robot = DhArm(dh_rr, Pose3());
  EXPECT_LONGS_EQUAL(2, robot.numLinks());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
