/**
 * @file  testUtils.cpp
 * @brief test utils functions
 * @Author: Mandy Xie
 */

#include <gtsam/base/numericalDerivative.h>
#include <utils.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test unit_twist function
 */
TEST(utils, unit_twist) {
  Vector3 w(0, 0, 1);
  Vector3 p(1, 0, 0);
  Vector6 expected_twist = (Vector(6) << 0, 0, 1, 0, -1, 0).finished();
  auto actual_twist = unit_twist(w, p);
  EXPECT(assert_equal(expected_twist, actual_twist, 1e-6));
}

/**
 * Test calculate system transition matrix for GP
 */
TEST(utils, calcPhi) {
  double t = 0.1;
  Matrix expected_phi(3, 3);
  expected_phi(0, 0) = 1;
  expected_phi(0, 1) = t;
  expected_phi(0, 2) = 0.5 * t * t;
  expected_phi(1, 0) = 0;
  expected_phi(1, 1) = 1;
  expected_phi(1, 2) = t;
  expected_phi(2, 0) = 0;
  expected_phi(2, 1) = 0;
  expected_phi(2, 2) = 1;
  auto actual_phi = calcPhi(t);
  EXPECT(assert_equal(expected_phi, actual_phi, 1e-6));
}

/**
 * Test calculate covariance matrix for GP
 */
TEST(utils, calcQ) {
  double t = 0.1;
  Matrix Qc = Matrix::Identity(1, 1);
  Matrix expected_Q(3*Qc.rows(), 3*Qc.cols());
  insertSub(expected_Q, 1.0 / 20 * pow(t, 5.0) * Qc, 0, 0);
  insertSub(expected_Q, 1.0 / 8 * pow(t, 4.0) * Qc, 0, 1);
  insertSub(expected_Q, 1.0 / 6 * pow(t, 3.0) * Qc, 0, 2);
  insertSub(expected_Q, 1.0 / 8 * pow(t, 4.0) * Qc, 1, 0);
  insertSub(expected_Q, 1.0 / 3 * pow(t, 3.0) * Qc, 1, 1);
  insertSub(expected_Q, 1.0 / 2 * pow(t, 2.0) * Qc, 1, 2);
  insertSub(expected_Q, 1.0 / 6 * pow(t, 3.0) * Qc, 2, 0);
  insertSub(expected_Q, 1.0 / 2 * pow(t, 2.0) * Qc, 2, 1);
  insertSub(expected_Q, t * Qc, 2, 2);
  
  auto actual_Q = calcQ(Qc, t);
  EXPECT(assert_equal(expected_Q, actual_Q, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}