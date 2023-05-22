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
#include <gtdynamics/factors/ContactPointFactor.h>
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

auto kPointModel = noiseModel::Isotropic::Sigma(3, 0.1);
auto kPoseModel = noiseModel::Isotropic::Sigma(6, 0.1);

auto robot = simple_rr::getRobot();

TEST(ContactPointFactor, Constructor) {
  Key link_pose_key = gtdynamics::PoseKey(0, 0),
      point_key = gtdynamics::PoseKey(1, 0);
  Point3 lPc(0, 0, 1);
  ContactPointFactor(link_pose_key, point_key, kPointModel, lPc);

  PointOnLink point_on_link(robot.links()[0], lPc);
  ContactPointFactor(point_on_link, point_key, kPointModel, 10);
}

TEST(ContactPointFactor, Error) {
  LinkSharedPtr end_link = robot.links()[0];
  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};

  Key point_key = gtdynamics::PoseKey(1, 0);
  Point3 wPc(0, 0, 1);

  ContactPointFactor factor(point_on_link, point_key, kPointModel, 0);

  Vector error = factor.evaluateError(point_on_link.link->bMcom(), wPc);
  EXPECT(assert_equal(Vector3(0, 0, -0.1), error, 1e-9));

  // Check error when contact point is not consistent
  Point3 wPc2(1, 1, 2);
  error = factor.evaluateError(point_on_link.link->bMcom(), wPc2);
  EXPECT(assert_equal(Vector3(1.0, 1.0, 0.9), error, 1e-9));
}

TEST(ContactPointFactor, Jacobians) {
  auto end_link = robot.links()[0];
  PointOnLink point_on_link{end_link, Point3(0, 0, 1)};

  Key point_key = gtdynamics::PoseKey(1, 0);
  Point3 wPc(0, 0, 1);

  ContactPointFactor factor(point_on_link, point_key, kPointModel, 0);

  Values values;
  InsertPose(&values, end_link->id(), 0, point_on_link.link->bMcom());
  values.insert<Point3>(point_key, wPc);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

TEST(FixedContactPointFactor, Error) {
  LinkSharedPtr end_link = robot.links()[0];
  Point3 contact_in_com(0, 0, 1);
  PointOnLink point_on_link{end_link, contact_in_com};

  Key link_pose_key = gtdynamics::PoseKey(0, 0);
  Point3 wPc(0, 0, 1);

  FixedContactPointFactor factor(link_pose_key, kPointModel, wPc, contact_in_com);
  Vector error = factor.evaluateError(point_on_link.link->bMcom());
  EXPECT(assert_equal(Vector3(0, 0, -0.1), error, 1e-9));

  // Check error when contact point is not consistent
  Point3 wPc2(1, 1, 2);
  FixedContactPointFactor factor2(link_pose_key, kPointModel, wPc2, contact_in_com);
  error = factor2.evaluateError(point_on_link.link->bMcom());
  EXPECT(assert_equal(Vector3(1.0, 1.0, 0.9), error, 1e-9));
}

TEST(FixedContactPointFactor, Jacobians) {
  auto end_link = robot.links()[0];
  Key link_pose_key = gtdynamics::PoseKey(0, 0);
  Point3 contact_in_com(0, 0, 1);
  PointOnLink point_on_link{end_link, contact_in_com};

  Point3 wPc(0, 0, 1);

  FixedContactPointFactor factor(link_pose_key, kPointModel, wPc, contact_in_com);

  Values values;
  InsertPose(&values, end_link->id(), 0, point_on_link.link->bMcom());

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

TEST(ContactPoseFactor, Constructor) {
  Key link_pose_key = gtdynamics::PoseKey(0, 0),
      point_key = gtdynamics::PoseKey(1, 0);
  Point3 wPc(0, 0, 1);
  Pose3 linkTcontact(Rot3(), wPc);
  ContactPoseFactor(link_pose_key, point_key, kPoseModel, linkTcontact);

  PointOnLink point_on_link(robot.links()[0], wPc);
  ContactPoseFactor(point_on_link, point_key, kPoseModel, 10);
}

TEST(ContactPoseFactor, Error) {
  LinkSharedPtr end_link = robot.links()[0];
  Point3 wPc(0, 0, 1);
  PointOnLink point_on_link{end_link, wPc};

  Key point_key = gtdynamics::PoseKey(1, 0);

  Pose3 wTcontact(Rot3(), wPc);

  ContactPoseFactor factor(point_on_link, point_key, kPoseModel, 0);

  Vector error = factor.evaluateError(point_on_link.link->bMcom(), wTcontact);
  Vector6 expected;
  expected << 0, 0, 0, 0, 0, -0.1;
  EXPECT(assert_equal(expected, error, 1e-9));

  // Check error when contact point is translated
  Point3 wPc2(1, 1, 2);
  Pose3 wTcontact2(Rot3(), wPc2);
  error = factor.evaluateError(point_on_link.link->bMcom(), wTcontact2);
  Vector6 expected2;
  expected2 << 0, 0, 0, 1.0, 1.0, 0.9;
  EXPECT(assert_equal(expected2, error, 1e-9));

  // Check error when contact point is rotated
  Pose3 wTcontact3(Rot3::RzRyRx(0, 2, 0), wPc);
  error = factor.evaluateError(point_on_link.link->bMcom(), wTcontact3);
  // regression
  Vector6 expected3;
  expected3 << 0, 2, -0, 0.1, 0, -0.0642092616;
  EXPECT(assert_equal(expected3, error, 1e-9));
}

TEST(ContactPoseFactor, Jacobians) {
  auto end_link = robot.links()[0];
  Point3 wPc(0, 0, 1);
  PointOnLink point_on_link{end_link, wPc};

  Key point_key = gtdynamics::PoseKey(1, 0);
  Pose3 wTc(Rot3(), wPc);

  ContactPoseFactor factor(point_on_link, point_key, kPoseModel, 0);

  Values values;
  InsertPose(&values, end_link->id(), 0, point_on_link.link->bMcom());
  values.insert<Pose3>(point_key, wTc);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  // Check Jacobians with non-identity rotation
  Pose3 wTc2(Rot3::RzRyRx(1, 2, 3), wPc);
  values.update<Pose3>(point_key, wTc2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
