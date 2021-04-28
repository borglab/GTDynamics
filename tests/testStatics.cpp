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
#include "gtdynamics/universal_robot/RevoluteJoint.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using namespace gtsam;
constexpr double kTol = 1e-6;

namespace example {
constexpr double g = 9.8;
const Robot robot = gtdynamics::CreateRobotFromFile(
    kSdfPath + std::string("/test/four_bar_linkage.sdf"));
Vector3 gravity(0, 0, -g);
}  // namespace example

TEST(Statics, GravityWrench1) {
  using namespace example;
  const Pose3 wTcom(Rot3(), Point3(1, 0, 0));
  const double mass = robot.link("l1")->mass();
  Matrix6 actualH;
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, 0, 0, -100 * g).finished(),
                      GravityWrench(gravity, mass, wTcom, actualH), kTol));
  Matrix6 numericalH = numericalDerivative11<Vector6, Pose3>(
      boost::bind(&GravityWrench, gravity, mass, _1, boost::none), wTcom);
  EXPECT(assert_equal(numericalH, actualH, kTol));
}

TEST(Statics, GravityWrench2) {
  using namespace example;
  const Pose3 wTcom(Rot3::Rx(M_PI_2), Point3(1, 0, 0));
  const double mass = robot.link("l2")->mass();
  Matrix6 actualH;
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, 0, -15 * g, 0).finished(),
                      GravityWrench(gravity, mass, wTcom, actualH), kTol));
  Matrix6 numericalH = numericalDerivative11<Vector6, Pose3>(
      boost::bind(&GravityWrench, gravity, mass, _1, boost::none), wTcom);
  EXPECT(assert_equal(numericalH, actualH, kTol));
}

TEST(Statics, ResultantWrench) {
  std::vector<Vector6> wrenches(2);
  wrenches[0] << 1, 2, 3, 4, 5, 6;
  wrenches[1] << 6, 5, 4, 3, 2, 1;
  std::vector<Matrix> actualH(2);
  EXPECT(assert_equal((Vector(6) << 7, 7, 7, 7, 7, 7).finished(),
                      ResultantWrench(wrenches, actualH), kTol));
  Matrix expected(6, 6);
  expected.setIdentity();
  EXPECT(assert_equal(expected, actualH[0], kTol));
  EXPECT(assert_equal(expected, actualH[1], kTol));
}

TEST(Statics, SecondExampleFromWrenchesColab) {
  constexpr double g = 10, mass = 1;  // for nice round numbers.
  const Vector3 gravity(0, -g, 0);
  const Pose3 wTcom(Rot3::Rz(M_PI / 3), Point3(0, 0, 0));
  const Vector6 Fg_B = GravityWrench(gravity, mass, wTcom);
  EXPECT(assert_equal((Vector(6) << 0, 0, 0, -8.66025404, -5, 0).finished(),
                      Fg_B, kTol));

  // Create base and link
  // TODO(frank): #207 should not have to provide wTl to Link constructor
  const gtsam::Pose3 wTl;  // we don't care!
  constexpr double L = 2;  // meters
  const gtsam::Pose3 lTcom(Rot3(), Point3(L / 2, 0, 0));
  const auto I3 = Matrix3::Identity();  // inertia
  auto base = boost::make_shared<Link>(0, "base", 1e10, I3, Pose3(), Pose3());
  auto link = boost::make_shared<Link>(1, "link", 1.0, I3, wTl, lTcom);

  // Create joint
  constexpr unsigned char id = 1;
  // TODO(frank): #206 should not have to provide wTj to the joint constructor.
  const Pose3 wTj;
  // TODO(frank): #205 make JointParams last argument and provide default
  const JointParams parameters;
  const gtsam::Vector3 axis(0, 0, 1);
  RevoluteJoint joint(id, "joint1", wTj, base, link, parameters, axis);
  Vector6 screw_axis = joint.screwAxis(link);
  const double tau = screw_axis.dot(-Fg_B);
  EXPECT_DOUBLES_EQUAL(5, tau, kTol);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
