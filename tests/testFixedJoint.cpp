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
#include "gtdynamics/universal_robot/FixedJoint.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"

using namespace gtdynamics;
using gtsam::assert_equal, gtsam::Pose3, gtsam::Point3, gtsam::Rot3;

/**
 * Construct a Fixed joint via Parameters and ensure all values are as
 * expected.
 */
TEST(FixedJoint, Constructor) {
  auto simple_urdf =
      get_sdf(std::string(URDF_PATH) + "/a1.urdf");
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
