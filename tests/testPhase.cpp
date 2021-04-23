/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPhase.cpp
 * @brief Test Phase class.
 * @author: Frank Dellaert, Tarushree Gandhi, Disha Das
 */

#include <CppUnitLite/TestHarness.h>

#include <sstream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/Phase.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;

TEST(Phase, error) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/spider.sdf"), "spider");
  constexpr size_t num_time_steps = 20;
  Phase phase(num_time_steps);
  phase.addContactPoint("tarsus_1_L1", {1, 1, 1});
  phase.addContactPoint("tarsus_2_L2", {2, 2, 2});
  phase.addContactPoint("tarsus_3_L3", {3, 3, 3});
  EXPECT_LONGS_EQUAL(33, robot.numLinks());

  ContactPoint cp = phase.contactPoint("tarsus_3_L3");
  EXPECT(assert_equal(Point3(3, 3, 3), cp.point));

  ContactPoints cps = phase.contactPoints();
  EXPECT_LONGS_EQUAL(3, cps.size());
  EXPECT(assert_equal(Point3(1, 1, 1), cps["tarsus_1_L1"].point));
  EXPECT_LONGS_EQUAL(20, phase.numTimeSteps());

  // Check printing
  std::stringstream ss;
  ss << phase;
  EXPECT(
      "[tarsus_1_L1: {[1 1 1], 0}, tarsus_2_L2: {[2 2 2], 0},"
      " tarsus_3_L3: {[3 3 3], 0}, ]" == ss.str());
}

TEST(Phase, inverse_kinematics) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");
  EXPECT_LONGS_EQUAL(13, robot.numLinks());

  constexpr size_t num_time_steps = 5;
  gtdynamics::Phase phase(num_time_steps);
  const Point3 contact_in_com(0.14, 0, 0);
  phase.addContactPoint("lower1", contact_in_com);  // LH
  phase.addContactPoint("lower0", contact_in_com);  // LF
  phase.addContactPoint("lower2", contact_in_com);  // RF
  phase.addContactPoint("lower3", contact_in_com);  // RH

  // Set goal points to reasonable values
  Point3 goal_LH(0, 0.15, 0);     // LH
  Point3 goal_LF(0.6, 0.15, 0);   // LF
  Point3 goal_RF(0.6, -0.15, 0);  // RF
  Point3 goal_RH(0, -0.15, 0);    // RH

  const gtsam::SharedNoiseModel cost_model = nullptr;
  const size_t k = 777;
  gtsam::NonlinearFactorGraph factors =
      phase.stanceObjectives(robot,
                             {{"lower1", goal_LH},
                              {"lower0", goal_LF},
                              {"lower2", goal_RF},
                              {"lower3", goal_RH}},
                             cost_model, k);
  EXPECT_LONGS_EQUAL(num_time_steps * 4, factors.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
