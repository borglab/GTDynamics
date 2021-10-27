/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testRevoluteJoint.cpp
 * @brief Test Joint class.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Matrix61;
using gtsam::Matrix66;
using gtsam::numericalDerivative11;
using gtsam::numericalDerivative21;
using gtsam::numericalDerivative22;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;

using namespace gtdynamics;

auto robot = simple_urdf::getRobot();
auto l1 = robot.link("l1");
auto l2 = robot.link("l2");

JointParams getJointParams() {
  JointParams parameters;
  parameters.effort_type = JointEffortType::Actuated;
  parameters.scalar_limits.value_lower_limit = -1.57;
  parameters.scalar_limits.value_upper_limit = 1.57;
  parameters.scalar_limits.value_limit_threshold = 0;
  return parameters;
}

/**
 * Construct a Revolute joint via Parameters and ensure all values are as
 * expected.
 */
TEST(Joint, Constructor) {
  JointParams parameters = getJointParams();

  const Vector3 axis(1, 0, 0);

  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, axis,
                   parameters);

  // name
  EXPECT(assert_equal(j1.name(), "j1"));

  // joint effort type
  EXPECT(j1.parameters().effort_type == JointEffortType::Actuated);

  // other link
  EXPECT(j1.otherLink(l2) == l1);
  EXPECT(j1.otherLink(l1) == l2);

  // rest transform
  Pose3 M21(Rot3::Rx(0), Point3(0, 0, -2)), M12 = M21.inverse();
  EXPECT(assert_equal(M21, j1.relativePoseOf(l1, 0.0)));
  EXPECT(assert_equal(M12, j1.relativePoseOf(l2, 0.0)));

  Pose3 wT2(Rot3::Rx(5), Point3(6, 7, 8));
  EXPECT(assert_equal(wT2 * M21, j1.poseOf(l1, wT2, 0.0)));

  Pose3 wT1(Rot3::Rx(4), Point3(1, 2, 3));
  EXPECT(assert_equal(wT1 * M12, j1.poseOf(l2, wT1, 0.0)));
}

// Test relativePoseOf and it derivatives
TEST(Joint, RelativePoseOfDerivatives) {
  const Vector3 axis(1, 0, 0);
  JointParams parameters = getJointParams();

  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, axis,
                   parameters);

  // Rotating joint by -M_PI / 2
  double q = -M_PI / 2;
  Pose3 T12(Rot3::Rx(q), Point3(0, 1, 1));
  Pose3 T21(Rot3::Rx(-q), Point3(0, 1, -1));

  // Calculate numerical derivatives of relativePoseOf.
  auto f1 = [&](double q) { return j1.relativePoseOf(l1, q); };
  Matrix61 numericalH1 = numericalDerivative11<Pose3, double>(f1, q);
  auto f2 = [&](double q) { return j1.relativePoseOf(l2, q); };
  Matrix61 numericalH2 = numericalDerivative11<Pose3, double>(f2, q);

  // Check relativePoseOf with derivatives.
  Matrix61 H1, H2;
  EXPECT(assert_equal(T21, j1.relativePoseOf(l1, q, H1)));
  EXPECT(assert_equal(T12, j1.relativePoseOf(l2, q, H2)));
  EXPECT(assert_equal(numericalH1, H1));
  EXPECT(assert_equal(numericalH2, H2));
}

// Test poseOf and it derivatives
TEST(Joint, PoseOfDerivatives) {
  const Vector3 axis(1, 0, 0);
  JointParams parameters = getJointParams();

  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, axis,
                   parameters);
  Pose3 wT1(Rot3::Rx(4), Point3(1, 2, 3));
  Pose3 wT2(Rot3::Rx(5), Point3(6, 7, 8));

  // Rotating joint by -M_PI / 2
  double q = -M_PI / 2;
  Pose3 T12(Rot3::Rx(q), Point3(0, 1, 1));
  Pose3 T21(Rot3::Rx(-q), Point3(0, 1, -1));

  // Calculate numerical derivatives of poseOf with respect to q and link pose.
  auto g1 = [&](const Pose3& wTl, double q) { return j1.poseOf(l1, wTl, q); };
  Matrix66 numericalH11 =
      numericalDerivative21<Pose3, Pose3, double>(g1, wT2, q);
  Matrix61 numericalH12 =
      numericalDerivative22<Pose3, Pose3, double>(g1, wT2, q);
  auto g2 = [&](const Pose3& wTl, double q) { return j1.poseOf(l2, wTl, q); };
  Matrix66 numericalH21 =
      numericalDerivative21<Pose3, Pose3, double>(g2, wT1, q);
  Matrix61 numericalH22 =
      numericalDerivative22<Pose3, Pose3, double>(g2, wT1, q);

  // Check poseOf with derivatives.
  Matrix61 H12, H22;
  Matrix66 H11, H21;
  EXPECT(assert_equal(wT2 * T21, j1.poseOf(l1, wT2, q, H11, H12)));
  EXPECT(assert_equal(wT1 * T12, j1.poseOf(l2, wT1, q, H21, H22)));
  EXPECT(assert_equal(numericalH11, H11));
  EXPECT(assert_equal(numericalH21, H21));
  EXPECT(assert_equal(numericalH12, H12));
  EXPECT(assert_equal(numericalH22, H22));
}

// Check values-based relativePoseOf, with derivatives.
TEST(Joint, ValuesRelativePoseOf) {
  const Vector3 axis(1, 0, 0);
  JointParams parameters = getJointParams();

  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, axis,
                   parameters);
  // Rotating joint by -M_PI / 2
  double q = -M_PI / 2;
  Pose3 T12(Rot3::Rx(q), Point3(0, 1, 1));
  Pose3 T21(Rot3::Rx(-q), Point3(0, 1, -1));

  // Calculate numerical derivatives of relativePoseOf with respect to other
  // link pose.
  auto f1 = [&](double q) { return j1.relativePoseOf(l1, q); };
  Matrix61 numericalH1 = numericalDerivative11<Pose3, double>(f1, q);
  auto f2 = [&](double q) { return j1.relativePoseOf(l2, q); };
  Matrix61 numericalH2 = numericalDerivative11<Pose3, double>(f2, q);

  Matrix H1v, H2v;
  EXPECT(assert_equal(T21, j1.relativePoseOf(l1, q, H1v)));
  EXPECT(assert_equal(T12, j1.relativePoseOf(l2, q, H2v)));
  EXPECT(assert_equal(numericalH1, H1v));
  EXPECT(assert_equal(numericalH2, H2v));

  // screw axis
  Vector6 screw_axis_l1, screw_axis_l2;
  screw_axis_l1 << -1, 0, 0, 0, -1, 0;
  screw_axis_l2 << 1, 0, 0, 0, -1, 0;
  EXPECT(assert_equal(screw_axis_l1, j1.screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_l2, j1.screwAxis(l2)));

  // links
  auto links = j1.links();
  EXPECT(links[0] == l1);
  EXPECT(links[1] == l2);

  // parent & child link
  EXPECT(j1.parent() == l1);
  EXPECT(j1.child() == l2);

  // joint limit
  EXPECT(assert_equal(parameters.scalar_limits.value_lower_limit,
                      j1.parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(parameters.scalar_limits.value_upper_limit,
                      j1.parameters().scalar_limits.value_upper_limit));
  EXPECT(assert_equal(parameters.scalar_limits.value_limit_threshold,
                      j1.parameters().scalar_limits.value_limit_threshold));
}

// Test parentTchild method at rest configuration.
TEST(RevoluteJoint, ParentTchild) {
  auto robot = simple_urdf::getRobot();
  auto j1 = robot.joint("j1");

  Pose3 pTc = j1->parentTchild(M_PI_2);
  // Rotate around the x axis for arm point up.
  // This means the second link bends to the right.
  Pose3 expected_pTc(Rot3::Rx(M_PI_2), Point3(0, -1, 1));
  EXPECT(assert_equal(expected_pTc, pTc, 1e-4));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
