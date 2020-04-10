/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testUtils.cpp
 * @brief Test utils functions.
 * @Author: Mandy Xie and Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <ignition/math/Pose3.hh>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <string>

#include "gtdynamics/utils/utils.h"

using gtsam::assert_equal;

// Test unit_twist function
TEST(utils, unit_twist) {
  gtsam::Vector3 w(0, 0, 1);
  gtsam::Vector3 p(1, 0, 0);
  gtsam::Vector6 expected_twist =
      (gtsam::Vector(6) << 0, 0, 1, 0, -1, 0).finished();
  auto actual_twist = gtdynamics::unit_twist(w, p);
  EXPECT(assert_equal(expected_twist, actual_twist, 1e-6));
}

// Test calculate system transition matrix for GP
TEST(utils, calcPhi) {
  double t = 0.1;
  gtsam::Matrix expected_phi =
      (gtsam::Matrix(3, 3) << 1, t, 0.5 * t * t, 0, 1, t, 0, 0, 1).finished();
  auto actual_phi = gtdynamics::calcPhi(t);
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

  auto actual_Q = gtdynamics::calcQ(Qc, t);
  EXPECT(assert_equal(expected_Q, actual_Q, 1e-6));
}

// Load a URDF file and ensure its joints and links were parsed correctly.
TEST(utils, load_and_parse_urdf_file) {
  // Load the file and parse URDF structure.
  auto simple_urdf =
      gtdynamics::get_sdf(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  // Check that physical and inertial properties were properly parsed..
  EXPECT(assert_equal(2, simple_urdf.LinkCount()));
  EXPECT(assert_equal(1, simple_urdf.JointCount()));

  EXPECT(assert_equal(
      100, simple_urdf.LinkByName("l1")->Inertial().MassMatrix().Mass()));
  EXPECT(assert_equal(
      15, simple_urdf.LinkByName("l2")->Inertial().MassMatrix().Mass()));

  EXPECT(assert_equal(3, simple_urdf.LinkByName("l1")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l1")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(1, simple_urdf.LinkByName("l1")->Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(1, simple_urdf.LinkByName("l2")->Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(2, simple_urdf.LinkByName("l2")->Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(3, simple_urdf.LinkByName("l2")->Inertial().Moi()(2, 2)));
}

TEST(utils, load_and_parse_sdf_file) {
  auto simple_sdf =
      gtdynamics::get_sdf(std::string(SDF_PATH) + "/test/simple.sdf");

  EXPECT(assert_equal(1, simple_sdf.LinkCount()));
  EXPECT(assert_equal(0, simple_sdf.JointCount()));
}

TEST(utils, load_and_parse_sdf_world_file) {
  auto simple_sdf = gtdynamics::get_sdf(
      std::string(SDF_PATH) + "/test/simple_rr.sdf", "simple_rr_sdf");

  EXPECT(assert_equal(3, simple_sdf.LinkCount()));
  EXPECT(assert_equal(2, simple_sdf.JointCount()));

  sdf::Link l0 = *simple_sdf.LinkByName("link_0");
  sdf::Link l1 = *simple_sdf.LinkByName("link_1");

  EXPECT(assert_equal(0.05, l0.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l0.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l0.Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(0.05, l1.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l1.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l1.Inertial().Moi()(2, 2)));
}

TEST(utils, parse_ignition_pose) {
  ignition::math::Pose3d pose_to_parse(-1, 1, -1, M_PI / 2, 0, -M_PI);

  gtsam::Pose3 parsed_pose = gtdynamics::parse_ignition_pose(
    pose_to_parse);

  EXPECT(assert_equal(
    gtsam::Pose3(
      gtsam::Rot3::RzRyRx(M_PI / 2, 0, -M_PI),
      gtsam::Point3(-1, 1, -1)),
    parsed_pose));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
