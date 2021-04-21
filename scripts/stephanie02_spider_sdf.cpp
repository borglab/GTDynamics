/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  stephanie02_spider_sdf.cpp
 * @brief Test reading a spider sdf file and doing something with it.
 * @author Stephanie McCormick
 */

#include <string>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/sdf.h"

using namespace gtdynamics;

int main(int argc, char** argv) {
  const Robot spider =
      CreateRobotFromFile(kSdfPath + "/test/spider.sdf", "spider");

  spider.print();

  return 0;
}
