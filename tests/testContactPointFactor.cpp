/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testContactPointFactor.cpp
 * @brief Test ContactPointFactor.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

auto kModel = noiseModel::Isotropic::Sigma(3, 0.1);

auto robot = simple_rr::getRobot();

TEST(ContactPointFactor, Constructor) {
  Key link_pose_key = gtdynamics::PoseKey(0, 0),
      point_key = gtdynamics::PoseKey(1, 0);
  Point3 lPc(0, 0, 1);
  ContactPointFactor(link_pose_key, point_key, kModel, lPc);

  PointOnLink point_on_link(robot.links()[0], lPc);
  ContactPointFactor(point_on_link, point_key, kModel, 10);
}

TEST(ContactPointFactor, Error) {
  LinkSharedPtr end_link = robot.links()[0];
  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};

  Key point_key = gtdynamics::PoseKey(1, 0);
  Point3 wPc(0, 0, 1);

  ContactPointFactor factor(point_on_link, point_key, kModel, 0);

  Vector error = factor.evaluateError(point_on_link.link->bMcom(), wPc);
  EXPECT(assert_equal(Vector3(0,0,-0.1), error, 1e-9));

  // Check error when contact point is not consistent
  Point3 wPc2(1, 1, 2);
  error = factor.evaluateError(point_on_link.link->bMcom(), wPc2);
  EXPECT(assert_equal(Vector3(1.0,1.0,0.9), error, 1e-9));
}

TEST(ContactPointFactor, Jacobians) {
  auto end_link = robot.links()[0];
  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};

  Key point_key = gtdynamics::PoseKey(1, 0);
  Point3 wPc(0, 0, 1);

  ContactPointFactor factor(point_on_link, point_key, kModel, 0);

  Values values;
  InsertPose(&values, end_link->id(), 0, point_on_link.link->bMcom());
  values.insert<Point3>(point_key, wPc);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
