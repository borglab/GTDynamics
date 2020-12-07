/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPhase.cpp
 * @brief Test Phase class.
 * @Author: Frank Dellaert, Tarushree Gandhi, Disha Das
 */

#include <CppUnitLite/TestHarness.h>
#include "gtdynamics/utils/Phase.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

using namespace gtdynamics; 

TEST(Phase, error) 
{
    Robot robot_configuration =
        CreateRobotFromFile(std::string(SDF_PATH) + "/test/spider.sdf", "spider");
    size_t num_time_steps = 20;
    auto phase = gtdynamics::Phase(robot_configuration, num_time_steps);
    double contact_height = 5;
    phase.addContactPoint("tarsus_1" , gtsam::Point3(3,3,3), contact_height);
    phase.addContactPoint("tarsus_2" , gtsam::Point3(3,3,3), contact_height);
    phase.addContactPoint("tarsus_3" , gtsam::Point3(3,3,3), contact_height);
    phase.addContactPoint("tarsus_3" , gtsam::Point3(2,2,2), contact_height + 5);
    auto robot = phase.getRobotConfiguration();
    EXPECT(robot.numLinks() == 33);

    ContactPoint cp = phase.getContactPointAtLink("tarsus_3");
    EXPECT(cp.name == "tarsus_3");
    
    ContactPoints cps = phase.getAllContactPoints();
    EXPECT(cps.size() == 3);
    EXPECT(cps[0].name == "tarsus_1");

    phase.generateContactPoints({"tarsus_6", "tarsus_7"}, 
                            gtsam::Point3(3,3,3), contact_height);
    ContactPoints cps_2 = phase.getAllContactPoints();
    EXPECT(cps_2.size() == 5);
    EXPECT(phase.numTimeSteps() == 20);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
