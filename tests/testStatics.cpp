/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testStatics.cpp
 * @brief Test calculations for statics.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind.hpp>
#include <cmath>

#include "gtdynamics/statics/Statics.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using namespace gtsam;

namespace example {
constexpr double g = 9.8;
const Robot robot = gtdynamics::CreateRobotFromFile(
    kSdfPath + std::string("/test/four_bar_linkage.sdf"));
Vector3 gravity(0, 0, -g);
}  // namespace example

TEST(WrenchFactor, Case1) {
  using namespace example;
  const Pose3 wTcom(Rot3(), Point3(1, 0, 0));
  const Matrix6 inertia = robot.link("l1")->inertiaMatrix();
  Matrix6 actualH;
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, 0, 0, -100 * g).finished(),
                      GravityWrench(gravity, inertia, wTcom, actualH), 1e-6));
  Matrix6 numericalH = numericalDerivative11<Vector6, Pose3>(
      boost::bind(&GravityWrench, gravity, inertia, _1, boost::none), wTcom);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

TEST(WrenchFactor, Case2) {
  using namespace example;
  const Pose3 wTcom(Rot3::Rx(M_PI_2), Point3(1, 0, 0));
  const Matrix6 inertia = robot.link("l2")->inertiaMatrix();
  Matrix6 actualH;
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, 0, -15 * g, 0).finished(),
                      GravityWrench(gravity, inertia, wTcom, actualH), 1e-6));
  Matrix6 numericalH = numericalDerivative11<Vector6, Pose3>(
      boost::bind(&GravityWrench, gravity, inertia, _1, boost::none), wTcom);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
