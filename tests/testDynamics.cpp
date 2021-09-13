/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamics.cpp
 * @brief Test calculations for statics.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind.hpp>
#include <cmath>

#include "gtdynamics/dynamics/Dynamics.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using namespace gtsam;

namespace example {
constexpr double g = 9.8;
const Robot robot = gtdynamics::CreateRobotFromFile(
    kSdfPath + std::string("/test/four_bar_linkage.sdf"));
Vector3 gravity(0, 0, -g);
}  // namespace example

TEST(Dynamics, Coriolis) {
  using namespace example;
  auto inertia = robot.link("l1")->inertiaMatrix();
  Matrix6 actualH;
  auto twist = (Vector(6) << 1, 2, 3, 4, 5, 6).finished();
  const Vector6 expected =
      gtsam::Pose3::adjointTranspose(twist, inertia * twist);
  EXPECT(assert_equal(expected, Coriolis(inertia, twist, actualH), 1e-6));
  Matrix6 numericalH = numericalDerivative11<Vector6, Vector6>(
      boost::bind(&Coriolis, inertia, _1, boost::none), twist);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
