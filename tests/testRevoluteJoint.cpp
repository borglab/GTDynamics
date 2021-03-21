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
using gtsam::numericalDerivative11;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;

using namespace gtdynamics;

/**
 * Construct a Revolute joint via Parameters and ensure all values are as
 * expected.
 */
TEST(Joint, params_constructor) {
  using simple_urdf::robot;
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  JointParams parameters;
  parameters.effort_type = JointEffortType::Actuated;
  parameters.scalar_limits.value_lower_limit = -1.57;
  parameters.scalar_limits.value_upper_limit = 1.57;
  parameters.scalar_limits.value_limit_threshold = 0;

  const Vector3 axis = (Vector(3) << 1, 0, 0).finished();

  RevoluteJoint j1(1, "j1", Pose3(Rot3(), Point3(0, 0, 2)), l1, l2, parameters,
                   axis);

  // name
  EXPECT(assert_equal(j1.name(), "j1"));

  // joint effort type
  EXPECT(j1.parameters().effort_type == JointEffortType::Actuated);

  // other link
  EXPECT(j1.otherLink(l2) == l1);
  EXPECT(j1.otherLink(l1) == l2);

  // Rotating joint by -M_PI / 2
  double q = -M_PI / 2;
  Pose3 T_12com(Rot3::Rx(q), Point3(0, 1, 1));
  Pose3 T_21com(Rot3::Rx(-q), Point3(0, 1, -1));

  // rest transform
  Pose3 T_21comRest(Rot3::Rx(0), Point3(0, 0, -2));
  EXPECT(assert_equal(T_21comRest, j1.transformTo(l2, 0.0)));
  EXPECT(assert_equal(T_21comRest.inverse(), j1.transformFrom(l2, 0.0)));

  // transform from
  EXPECT(assert_equal(T_12com, j1.transformFrom(l2, q)));
  EXPECT(assert_equal(T_21com, j1.transformFrom(l1, q)));

  // Calculate enumerical derivatives of transformTo.
  auto f1 = [&](double q) { return j1.transformTo(l1, q); };
  Matrix61 numericalH1 = numericalDerivative11<Pose3, double>(f1, q);
  auto f2 = [&](double q) { return j1.transformTo(l2, q); };
  Matrix61 numericalH2 = numericalDerivative11<Pose3, double>(f2, q);

  // Check transformTo with derivatives.
  Matrix61 H1, H2;
  EXPECT(assert_equal(T_12com, j1.transformTo(l1, q, H1)));
  EXPECT(assert_equal(T_21com, j1.transformTo(l2, q, H2)));
  EXPECT(assert_equal(numericalH1, H1));
  EXPECT(assert_equal(numericalH2, H2));

  // Check values-based TransformTo, with derivatives.
  Values values;
  const size_t t = 0;
  InsertJointAngle(&values, 1, q);
  Matrix H1v, H2v;
  EXPECT(assert_equal(T_12com, j1.transformTo(t, l1, values, H1v)));
  EXPECT(assert_equal(T_21com, j1.transformTo(t, l2, values, H2v)));
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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
