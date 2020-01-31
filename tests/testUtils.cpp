/**
 * @file  testUtils.cpp
 * @brief test utils functions
 * @Author: Mandy Xie and Alejandro Escontrela
 */

#include <gtsam/base/numericalDerivative.h>
#include <utils.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <string>
#include <limits.h>
#include <unistd.h>
#include <algorithm>

using namespace std;
using namespace gtsam;
using namespace manipulator;
using namespace robot;

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

// Load a URDF file and ensure its joints and links were parsed correctly.
TEST(utils, load_and_parse_urdf_file) {
  // Load the file and parse URDF structure.
  auto simple_urdf = get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  // Check that physical and inertial properties were properly parsed..
  EXPECT(assert_equal(2, simple_urdf.LinkCount()));
  EXPECT(assert_equal(1, simple_urdf.JointCount()));

  EXPECT(assert_equal(100, simple_urdf.LinkByName("l1")->Inertial().MassMatrix().Mass()));
  EXPECT(assert_equal(15, simple_urdf.LinkByName("l2")->Inertial().MassMatrix().Mass()));

  EXPECT(assert_equal(3, simple_urdf.LinkByName("l1")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l1")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(1, simple_urdf.LinkByName("l1")->Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(1, simple_urdf.LinkByName("l2")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l2")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(3, simple_urdf.LinkByName("l2")->Inertial().Moi()(2, 2)));
}

TEST(utils, load_and_parse_sdf_file) {
  
  auto simple_sdf = get_sdf(std::string(SDF_PATH) + "/test/simple.sdf");

  EXPECT(assert_equal(1, simple_sdf.LinkCount()));
  EXPECT(assert_equal(0, simple_sdf.JointCount()));
}

TEST(utils, load_and_parse_sdf_world_file) {
  auto simple_sdf = get_sdf(std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  EXPECT(assert_equal(3, simple_sdf.LinkCount()));
  EXPECT(assert_equal(3, simple_sdf.JointCount()));

  sdf::Link l0 = *simple_sdf.LinkByName("link_0");
  sdf::Link l1 = *simple_sdf.LinkByName("link_1");

  EXPECT(assert_equal(0.05, l0.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l0.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l0.Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(0.05, l1.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l1.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l1.Inertial().Moi()(2, 2)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
