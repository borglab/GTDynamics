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

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/Phase.h"
#include <sstream>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;

TEST(Phase, error) {
  Robot robot =
      CreateRobotFromFile(kSdfPath + std::string("/test/spider.sdf"), "spider");
  constexpr size_t num_time_steps = 20;
  gtdynamics::Phase phase(robot, num_time_steps);
  constexpr double contact_height = 5;
  phase.addContactPoint("tarsus_1", Point3(1, 1, 1), contact_height);
  phase.addContactPoint("tarsus_2", Point3(2, 2, 2), contact_height);
  phase.addContactPoint("tarsus_3", Point3(3, 3, 3), contact_height);
  EXPECT_LONGS_EQUAL(33, robot.numLinks());

  ContactPoint cp = phase.getContactPointAtLink("tarsus_3");
  EXPECT(assert_equal(Point3(3, 3, 3), cp.point));

  ContactPoints cps = phase.contactPoints();
  EXPECT_LONGS_EQUAL(3, cps.size());
  EXPECT(assert_equal(Point3(1, 1, 1), cps["tarsus_1"].point));
  EXPECT_LONGS_EQUAL(20, phase.numTimeSteps());

  // Check printing
  std::stringstream ss;
  ss << phase;
  EXPECT("[tarsus_1: {[1 1 1], 0, 5}, tarsus_2: {[2 2 2], 0, 5},"
         " tarsus_3: {[3 3 3], 0, 5}, ]" == ss.str());
}

TEST(Phase, inverse_kinematics) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");

  constexpr size_t num_time_steps = 5;
  gtdynamics::Phase phase(robot, num_time_steps);
  EXPECT_LONGS_EQUAL(13, phase.robot().numLinks());

  // Set contacts to reasonable values
  phase.addContactPoint("lower1", Point3(0, 0.15, 0), 0);    // LH
  phase.addContactPoint("lower0", Point3(0.6, 0.15, 0), 0);  // LF
  phase.addContactPoint("lower2", Point3(0.6, -0.15, 0), 0);   // RF
  phase.addContactPoint("lower3", Point3(0, -0.15, 0), 0); // RF


}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
