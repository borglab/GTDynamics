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
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <ignition/math/Pose3.hh>
#include <string>

#include "gtdynamics/utils/utils.h"

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
