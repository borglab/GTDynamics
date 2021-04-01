/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWrenchFactor.cpp
 * @brief Test wrench factors.
 * @author Yetong Zhang
 */

#include "gtdynamics/factors/WrenchFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

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

using namespace gtdynamics;
using internal::TwistKey;
using internal::TwistAccelKey;
using internal::WrenchKey;
using internal::PoseKey;
using namespace gtsam;

namespace example {

// R link example
using simple_urdf_zero_inertia::robot;

auto inertia = robot.links()[0]->inertiaMatrix();
Vector3 gravity = (Vector3() << 0, -9.8, 0).finished();

noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);

}  // namespace example

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor, error2) {
  // Create all factors
  int id = 0;

  WrenchFactor factor(TwistKey(id), TwistAccelKey(id),
                      {WrenchKey(id, 1), WrenchKey(id, 2)},
                      PoseKey(id), example::cost_model, example::inertia,
                      example::gravity);
  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, -1, 0, 4.9, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 1, 0, 4.9, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor.unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, 1e-3);
}

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor, error3) {
  // Create all factors
  int id = 0;

  WrenchFactor factor(
      TwistKey(id), TwistAccelKey(id),
      {WrenchKey(id, 1), WrenchKey(id, 2), WrenchKey(id, 3)},
      PoseKey(id), example::cost_model, example::inertia, example::gravity);
  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 0, 0, 2, 0).finished());
  InsertWrench(&x, id, 3, (Vector(6) << 0, 0, 0, 0, 6.8, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor.unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, 1e-3);
}

// Test wrench factor for stationary case with gravity
TEST(WrenchFactor, error4) {
  // Create all factors
  int id = 0;

  WrenchFactor factor(TwistKey(id), TwistAccelKey(id),
                      {WrenchKey(id, 1), WrenchKey(id, 2),
                       WrenchKey(id, 3), WrenchKey(id, 4)},
                      PoseKey(id), example::cost_model, example::inertia,
                      example::gravity);
  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 3, (Vector(6) << 0, 0, 0, 0, 1, 0).finished());
  InsertWrench(&x, id, 4, (Vector(6) << 0, 0, 0, 0, 6.8, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor.unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, 1e-3);
}

// Test wrench factor for non-zero twist case, zero joint angle
TEST(WrenchFactor, error_nonzero) {
  // Create all factors
  int id = 0;

  WrenchFactor factor(TwistKey(id), TwistAccelKey(id),
                      {WrenchKey(id, 1), WrenchKey(id, 2)},
                      PoseKey(id), example::cost_model, example::inertia);

  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 1, 0, 1, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 1, 0, 1, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 4, -1, 2, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, -4, 0, -1, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor.unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
