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
  Robot robot_configuration =
      CreateRobotFromFile(kSdfPath + std::string("/test/spider.sdf"), "spider");
  size_t num_time_steps = 20;
  auto phase = gtdynamics::Phase(robot_configuration, num_time_steps);
  double contact_height = 5;
  phase.addContactPoint("tarsus_1", Point3(1, 1, 1), contact_height);
  phase.addContactPoint("tarsus_2", Point3(2, 2, 2), contact_height);
  phase.addContactPoint("tarsus_3", Point3(3, 3, 3), contact_height);
  auto robot = phase.getRobotConfiguration();
  EXPECT_LONGS_EQUAL(33, robot.numLinks());

  ContactPoint cp = phase.getContactPointAtLink("tarsus_3");
  EXPECT(assert_equal(Point3(3, 3, 3), cp.point));

  ContactPoints cps = phase.contactPoints();
  EXPECT_LONGS_EQUAL(3, cps.size());
  EXPECT(assert_equal(Point3(1, 1, 1), cps["tarsus_1"].point));
  EXPECT_LONGS_EQUAL(20, phase.numTimeSteps());

  // Check printing
  std::stringstream ss;
  EXPECT("[tarsus_1: {[1 1 1], 0, 5}, tarsus_2: {[2 2 2], 0, 5},"
         " tarsus_3: {[3 3 3], 0, 5}, ]" == ss.str());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
