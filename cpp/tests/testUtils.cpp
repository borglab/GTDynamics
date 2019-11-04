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

// Test unit_twist function
TEST(utils, unit_twist) {
  Vector3 w(0, 0, 1);
  Vector3 p(1, 0, 0);
  Vector6 expected_twist = (Vector(6) << 0, 0, 1, 0, -1, 0).finished();
  auto actual_twist = unit_twist(w, p);
  EXPECT(assert_equal(expected_twist, actual_twist, 1e-6));
}

// Test calculate system transition matrix for GP
TEST(utils, calcPhi) {
  double t = 0.1;
  Matrix expected_phi =
      (Matrix(3, 3) << 1, t, 0.5 * t * t, 0, 1, t, 0, 0, 1).finished();
  auto actual_phi = calcPhi(t);
  EXPECT(assert_equal(expected_phi, actual_phi, 1e-6));
}

// Test calculate covariance matrix for GP
TEST(utils, calcQ) {
  double t = 0.1;
  Matrix Qc = I_6x6;
  Matrix expected_Q =
      (Matrix(3 * Qc.rows(), 3 * Qc.rows()) << 1.0 / 20 * pow(t, 5.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 3 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, t * Qc)
          .finished();

  auto actual_Q = calcQ(Qc, t);
  EXPECT(assert_equal(expected_Q, actual_Q, 1e-6));
}

// Test readFromTxt
TEST(utils, readFromTxt) {
  Point3 expected_origin(-1, -1, -1), actual_origin;
  double expected_cell_size = 0.01, actual_cell_size;
  vector<Matrix> expected_data, actual_data;
  Matrix3 mat1;
  mat1 << 1, 2, 3,  //
      4, 5, 6,      //
      7, 8, 9;
  Matrix3 mat2;
  mat2 << 11, 22, 33,  //
      44, 55, 66,      //
      77, 88, 99;
  expected_data.push_back(mat1);
  expected_data.push_back(mat2);

  actual_data = readFromTxt("../../../matlab/dataset/test.txt", actual_origin,
                            actual_cell_size);
  EXPECT(assert_equal(expected_data, actual_data, 1e-6));
  EXPECT(assert_equal(expected_origin, actual_origin, 1e-6));
  EXPECT(assert_equal(expected_cell_size, actual_cell_size, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
