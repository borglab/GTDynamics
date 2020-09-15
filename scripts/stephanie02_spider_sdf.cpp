/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  stephanie02_spider_sdf.cpp
 * @brief Test reading a spider sdf file and doing something with it.
 * @Author: Stephanie McCormick
 */

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

#include <string>

using namespace gtdynamics; 

int main(int argc, char** argv) {
  const auto spider =
      CreateRobotFromFile(std::string(SDF_PATH) + "/test/spider.sdf", "spider");

  spider.printRobot();

  return 0;
}
