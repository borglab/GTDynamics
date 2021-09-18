/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPrismaticJoint.cpp
 * @brief Test Joint class.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include "gtdynamics/universal_robot/Link.h"
#include "gtdynamics/universal_robot/PrismaticJoint.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

/**
 * Construct a Prismatic joint via JointParams and ensure all values are as
 * expected.
 */
TEST(Joint, params_constructor_prismatic) {
  auto robot = simple_urdf_prismatic::getRobot();
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  JointParams parameters;
  parameters.effort_type = JointEffortType::Actuated;
  parameters.scalar_limits.value_lower_limit = 0;
  parameters.scalar_limits.value_upper_limit = 2;
  parameters.scalar_limits.value_limit_threshold = 0;

  const gtsam::Vector3 j1_axis = (gtsam::Vector(3) << 0, 0, 1).finished();

  auto j1 = boost::make_shared<PrismaticJoint>(
      1, "j1", Pose3(Rot3::Rx(1.5707963268), Point3(0, 0, 2)), l1, l2, j1_axis,
      parameters);

  // get shared ptr
  EXPECT(j1->shared() == j1);

  // get, set ID
  EXPECT(j1->id() == 1);

  // name
  EXPECT(assert_equal(j1->name(), "j1"));

  // joint effort type
  EXPECT(j1->parameters().effort_type == JointEffortType::Actuated);

  // other link
  EXPECT(j1->otherLink(l2) == l1);
  EXPECT(j1->otherLink(l1) == l2);

  // rest transform
  Pose3 T_12comRest(Rot3::Rx(1.5707963268), Point3(0, -1, 1));
  Pose3 T_21comRest(Rot3::Rx(-1.5707963268), Point3(0, -1, -1));
  EXPECT(assert_equal(T_12comRest, j1->relativePoseOf(l2, 0.0), 1e-5));
  EXPECT(assert_equal(T_21comRest, j1->relativePoseOf(l1, 0.0), 1e-5));

  // transform to (translating +1)
  Pose3 T_12com(Rot3::Rx(1.5707963268), Point3(0, -2, 1));
  Pose3 T_21com(Rot3::Rx(-1.5707963268), Point3(0, -1, -2));
  EXPECT(assert_equal(T_12com, j1->relativePoseOf(l2, 1), 1e-5));
  EXPECT(assert_equal(T_21com, j1->relativePoseOf(l1, 1), 1e-5));

  // screw axis
  gtsam::Vector6 screw_axis_l1, screw_axis_l2;
  screw_axis_l1 << 0, 0, 0, 0, 1, 0;
  screw_axis_l2 << 0, 0, 0, 0, 0, 1;
  EXPECT(assert_equal(screw_axis_l1, j1->screwAxis(l1)));
  EXPECT(assert_equal(screw_axis_l2, j1->screwAxis(l2), 1e-5));

  // links
  auto links = j1->links();
  EXPECT(links[0] == l1);
  EXPECT(links[1] == l2);

  // parent & child link
  EXPECT(j1->parent() == l1);
  EXPECT(j1->child() == l2);

  // joint limit
  EXPECT(assert_equal(parameters.scalar_limits.value_lower_limit,
                      j1->parameters().scalar_limits.value_lower_limit));
  EXPECT(assert_equal(parameters.scalar_limits.value_upper_limit,
                      j1->parameters().scalar_limits.value_upper_limit));
  EXPECT(assert_equal(parameters.scalar_limits.value_limit_threshold,
                      j1->parameters().scalar_limits.value_limit_threshold));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
