/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testContactEqualityFactor.cpp
 * @brief Test ContactEqualityFactor.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ContactEqualityFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

auto kModel = noiseModel::Isotropic::Sigma(3, 0.1);

auto robot = simple_rr::getRobot();

TEST(ContactEqualityFactor, Constructor) {
  PointOnLink point_on_link{robot.link("link_0"), Point3(0, 0, 1)};
  ContactEqualityFactor(point_on_link, kModel, 0, 1);
}

TEST(ContactEqualityFactor, Error) {
  LinkSharedPtr end_link = robot.links()[0];
  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};

  ContactEqualityFactor factor(point_on_link, kModel, 0, 1);

  // Hand computed value from SDF file
  Pose3 bTl1(Rot3(), Point3(0, 0, 0.1)), bTl2(Rot3(), Point3(0, 0, 0.1));
  Vector error = factor.evaluateError(bTl1, bTl2);

  EXPECT(assert_equal(Vector3::Zero(), error, 1e-9));

  // Check error when poses are not equal
  bTl2 = Pose3(Rot3(), Point3(1, 1, 1.1));
  error = factor.evaluateError(bTl1, bTl2);
  EXPECT(assert_equal(Vector3::Ones(), error, 1e-9));
}

TEST(ContactEqualityFactor, Jacobians) {
  auto end_link = robot.links()[0];

  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};
  ContactEqualityFactor factor(point_on_link, kModel, 0, 1);

  Pose3 bTl1(Rot3(), Point3(0, 0, 1));

  Values values;
  InsertPose(&values, end_link->id(), 0, bTl1);
  InsertPose(&values, end_link->id(), 1, bTl1);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

TEST(ContactEqualityFactor, ArbitraryTime) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  auto end_link = robot.link("l2");

  size_t k1 = 81, k2 = k1 + 3;
  PointOnLink point_on_link{end_link, Point3(0, 0, 0.777)};
  ContactEqualityFactor factor(point_on_link, kModel, k1, k2);

  Pose3 bTl1(Rot3(), Point3(1, 2, 3));
  Vector error = factor.evaluateError(bTl1, bTl1);
  EXPECT(assert_equal(Vector3::Zero(), error, 1e-9));

  // Check Jacobians
  Values values;
  InsertPose(&values, end_link->id(), k1, bTl1);
  InsertPose(&values, end_link->id(), k2, bTl1);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
