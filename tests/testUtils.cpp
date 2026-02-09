/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testUtils.cpp
 * @brief Test utils functions.
 * @author Mandy Xie and Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <gz/math/Pose3.hh>
#include <string>

using namespace gtdynamics;
using gtsam::assert_equal;

// Test unit_twist function
TEST(utils, unit_twist) {
  gtsam::Vector3 w(0, 0, 1);
  gtsam::Vector3 p(1, 0, 0);
  gtsam::Vector6 expected_twist =
      (gtsam::Vector(6) << 0, 0, 1, 0, -1, 0).finished();
  auto actual_twist = unit_twist(w, p);
  EXPECT(assert_equal(expected_twist, actual_twist, 1e-6));
}

// Test calculate system transition matrix for GP
TEST(utils, calcPhi) {
  double t = 0.1;
  gtsam::Matrix expected_phi =
      (gtsam::Matrix(3, 3) << 1, t, 0.5 * t * t, 0, 1, t, 0, 0, 1).finished();
  auto actual_phi = calcPhi(t);
  EXPECT(assert_equal(expected_phi, actual_phi, 1e-6));
}

// Test calculate covariance matrix for GP
TEST(utils, calcQ) {
  double t = 0.1;
  gtsam::Matrix Qc = gtsam::I_6x6;
  gtsam::Matrix expected_Q =
      (gtsam::Matrix(3 * Qc.rows(), 3 * Qc.rows())
           << 1.0 / 20 * pow(t, 5.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 3 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, t * Qc)
          .finished();

  auto actual_Q = calcQ(Qc, t);
  EXPECT(assert_equal(expected_Q, actual_Q, 1e-6));
}

TEST(utils, Derivatives) {
  using namespace gtsam;
  Point3 p(1, 2, 3);
  Matrix H_p;
  EXPECT_DOUBLES_EQUAL(3.0, point3_z(p, H_p), 1e-9);
  EXPECT(assert_equal((Matrix(1, 3) << 0, 0, 1).finished(), H_p));

  double x1 = 10.0, x2 = 2.0;
  Matrix H1, H2;
  EXPECT_DOUBLES_EQUAL(5.0, double_division(x1, x2, H1, H2), 1e-9);
  EXPECT(assert_equal((Matrix(1, 1) << 0.5).finished(), H1));
  EXPECT(assert_equal((Matrix(1, 1) << -2.5).finished(), H2));

  double x = 2.0;
  Matrix H;
  EXPECT_DOUBLES_EQUAL(0.5, reciprocal(x, H), 1e-9);
  EXPECT(assert_equal((Matrix(1, 1) << -0.25).finished(), H));

  EXPECT_DOUBLES_EQUAL(1.0, clip_by_one(0.5, H), 1e-9);
  EXPECT(assert_equal((Matrix(1, 1) << 0.0).finished(), H));
  EXPECT_DOUBLES_EQUAL(2.0, clip_by_one(2.0, H), 1e-9);
  EXPECT(assert_equal((Matrix(1, 1) << 1.0).finished(), H));

  Matrix H1s, H2s;
  EXPECT(assert_equal(Vector2(10.0, 2.0), double_stack(x1, x2, H1s, H2s)));
  EXPECT(assert_equal((Matrix(2, 1) << 1, 0).finished(), H1s));
  EXPECT(assert_equal((Matrix(2, 1) << 0, 1).finished(), H2s));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
