/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testChain.cpp
 * @brief Test Chain class.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include "gtdynamics/dynamics/Chain.h"

using namespace gtdynamics;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Matrix;
using gtsam::assert_equal;

TEST(threeLinks, All) {
  
  Pose3 sMb = Pose3(Rot3(), Point3(5, 0, 0));
  Matrix screwAxis(6,1);
  screwAxis << 0.0, 0.0, 1.0, 0.0, 5.0, 0.0;
  auto joint1 = Chain(sMb, screwAxis);
  auto joint2 = Chain(sMb, screwAxis);
  auto joint3 = Chain(sMb, screwAxis);
  std::vector<Chain> chain_vector;
  chain_vector.push_back(joint1);
  chain_vector.push_back(joint2);
  chain_vector.push_back(joint3);
  Chain res = Chain::compose(chain_vector);
  Pose3 expected = Pose3(Rot3(), Point3(15,0,0));
  Matrix expected_J(6,3);
  expected_J << 0,0,0,
                 0,0,0,
                 1,1,1,
                 0,0,0,
                 15,10,5,
                 0,0,0;
  EXPECT(assert_equal(res.sMb(), expected, 1e-6));
  EXPECT(assert_equal(res.axes(), expected_J, 1e-6));

  // FK at rest
  Vector joint_angles(3);
  joint_angles << 0,0,0;
  Matrix J0;
  Pose3 POE = res.poe(joint_angles, boost::none, J0);
  EXPECT(assert_equal(POE, expected, 1e-6));
  EXPECT(assert_equal(J0, expected_J, 1e-6));

  //FK not at rest
  Vector joint_angles1(3);
  joint_angles1 << 0,0,M_PI/2;
  Pose3 expected_not_rest = Pose3(Rot3::Rz(M_PI/2), Point3(10, 5, 0));
  Matrix J1;
  Matrix expected_J1(6,3);
  expected_J1 << 0,0,0,
                 0,0,0,
                 1,1,1,
                 10,5,0,
                 5,5,5,
                 0,0,0;
  Pose3 POE1 = res.poe(joint_angles1, boost::none, J1);
  EXPECT(assert_equal(POE1, expected_not_rest, 1e-6));
  EXPECT(assert_equal(J1, expected_J1, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}