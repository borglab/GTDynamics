/**
 * @file  testPandaIKFast.cpp
 * @brief test Panda Ikfast wrapper
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

TEST(PandaIKFast, Forward) {
  // Create PandaIKFast object
  PandaIKFast pandarobot = PandaIKFast();

  // Should this be put this into two different tests?
  // Simple case: all joints with value 0
  Vector7 joints_simple = (Vector7() << 0, 0, 0, 0, 0, 0, 0).finished();

  Rot3 expected_bRe_simple =
      Rot3((Matrix3() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished());
  Point3 expected_bte_simple = (Point3() << 0.088, 0, 1.033).finished();
  Pose3 expected_bTe_simple(expected_bRe_simple, expected_bte_simple);

  Pose3 actual_bTe_simple = pandarobot.forward(joints_simple);

  EXPECT(assert_equal(expected_bTe_simple, actual_bTe_simple, 1e-5))

  // Complex case: random joint values
  Vector7 joints = (Vector7() << -1.9205802374693666, -0.07431401149295525,
                    1.9902035554092978, -2.068096770889272, 2.64181363620466,
                    3.505368647887169, -1.1871823345782546)
                       .finished();

  Rot3 expected_bRe =
      Rot3((Matrix3() << 0.0224721, -0.747238, 0.664177, -0.967494, -0.18364,
            -0.173871, 0.251893, -0.63868, -0.727075)
               .finished());
  Point3 expected_bte =
      (Point3() << 0.403041, -0.00106015, 0.480041).finished();
  Pose3 expected_bTe(expected_bRe, expected_bte);

  Pose3 actual_bTe = pandarobot.forward(joints);

  EXPECT(assert_equal(expected_bTe, actual_bTe, 1e-5))
}

TEST(PandaIKFast, Inverse) {
  // Create PandaIKFast object
  PandaIKFast pandarobot = PandaIKFast();

  Rot3 bRe = Rot3((Matrix3() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished());
  Point3 bte = (Point3() << 0, 0, 0.25).finished();
  Pose3 bTe(bRe, bte);
  double theta7 = 0.3;

  std::vector<Vector7> expected_solutions(8);
  expected_solutions[0] << 2.84159, 0.581145, 3.14159, 2.3909, -3.14159,
      -0.169549, 0.3;
  expected_solutions[1] << -0.3, -0.581145, -3.55271e-15, 2.3909, -3.14159,
      -0.169549, 0.3;
  expected_solutions[2] << -0.3, 2.2104, 3.14159, 2.3909, 0, -1.45971, 0.3;
  expected_solutions[3] << 2.84159, -2.2104, -3.55271e-15, 2.3909, 0, -1.45971,
      0.3;
  expected_solutions[4] << 2.84159, 0.070394, -2.53556e-14, 2.95828,
      -7.10543e-15, 0.253705, 0.3;
  expected_solutions[5] << -0.3, -0.070394, 3.14159, 2.95828, -7.10543e-15,
      0.253705, 0.3;
  expected_solutions[6] << -0.3, 1.69965, -6.70103e-15, 2.95828, 3.14159,
      -1.88296, 0.3;
  expected_solutions[7] << 2.84159, -1.69965, 3.14159, 2.95828, 3.14159,
      -1.88296, 0.3;

  std::vector<Vector7> actual_solutions = pandarobot.inverse(bTe, theta7);

  EXPECT(assert_equal(8, actual_solutions.size()));

  // Check for every solution if they are equal
  for (size_t i = 0; i < 8; ++i) {
    EXPECT(assert_equal(expected_solutions[i], actual_solutions[i], 1e-5));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
