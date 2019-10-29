/**
 * @file  testDhArm.cpp
 * @brief test robotic arm with DH links
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <DhArm.h>
#include <DHLink.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// Unit tests for dh link RR
TEST(DhArm, DH_RR) {
  vector<DH_Link> dh_rr = {
      DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), I_3x3, -180, 180, 20),
      DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), I_3x3, -180, 180, 20)};

  auto robot = DhArm(dh_rr, Pose3());
  EXPECT_LONGS_EQUAL(2, robot.numLinks());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
