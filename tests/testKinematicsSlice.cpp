/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testKinematicsSlice.cpp
 * @brief Test Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/ContactPoint.h>

// #includ<">gtdynamics/kinematics/KinematicsSlice.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <vector>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

struct ContactGoal {
  PointOnLink point_on_link;  ///< In COM.
  gtsam::Point3 goal_point;   ///< In world frame.

  /// Return link associated with contact point
  const LinkSharedPtr& link() const { return point_on_link.link; }

  /**
   * @fn Check that the contact goal has been achived for given values.
   * @param values a GTSAM Values instance that should contain link pose.
   * @param k time step to check (default 0).
   * @param tol tolerance in 3D (default 1e-9).
   */
  bool satisfied(const gtsam::Values& values, size_t k = 0,
                 double tol = 1e-9) const {
    // Change point reference frame from com to spatial.
    const gtsam::Pose3 wTcom = Pose(values, link()->id(), k);
    const gtsam::Point3 point_w = wTcom.transformFrom(point_on_link.point);
    return gtsam::distance3(point_w, goal_point) < tol;
  }
};

///< Map of link name to ContactGoal
using ContactGoals = std::vector<ContactGoal>;

gtsam::Values InverseKinematics(const ContactGoals& contact_goals) {
  return gtsam::Values();
}

TEST(Phase, inverse_kinematics) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");

  // establish contact/goal pairs
  const Point3 contact_in_com(0.14, 0, 0);
  const ContactGoals contact_goals = {
      {{robot.link("lower1"), contact_in_com}, {0, 0.15, 0}},     // LH
      {{robot.link("lower0"), contact_in_com}, {0.6, 0.15, 0}},   // LF
      {{robot.link("lower2"), contact_in_com}, {0.6, -0.15, 0}},  // RF
      {{robot.link("lower3"), contact_in_com}, {0, -0.15, 0}}};   // RH

  // TODO(frank): consider renaming ContactPoint to PointOnLink
  auto result = InverseKinematics(contact_goals);

  // Check that goals are achieved
  for (const ContactGoal& contact_goal : contact_goals) {
    EXPECT(contact_goal.satisfied(result));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
