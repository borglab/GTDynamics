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

#include "gtdynamics/factors/WrenchFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;

namespace example {

// R link example
auto robot = simple_urdf::getRobot();
const auto link = robot.links()[0];
const Matrix6 inertia = link->inertiaMatrix();
const Vector3 gravity(0, -9.8, 0);

noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);

}  // namespace example

constexpr double diffDelta = 1e-4;
constexpr double tol = 1e-5;

// Test wrench factor for non-stationary case
TEST(WrenchFactor, Case1) {
  // Create all factors
  int id = 0;
  const double M = example::inertia(3, 3);
  auto factor = WrenchFactor(example::cost_model, example::link,
                      {WrenchKey(id, 1), WrenchKey(id, 2)},
                       0, example::gravity);
  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id,
                   (Vector(6) << 0, 0, 0, 0, -9.8 + 12 / M, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, -1, 0, 10, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 1, 0, 2, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor->unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, x, diffDelta, tol);
}

// Test wrench factor compensating exactly for gravity
TEST(WrenchFactor, Case2) {
  // Create all factors
  int id = 0;
  const double M = example::inertia(3, 3);
  auto factor = WrenchFactor(example::cost_model, example::link,
                      {WrenchKey(id, 1), WrenchKey(id, 2), WrenchKey(id, 3)},
                      0, example::gravity);
  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 0, 0, 0, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 0, 0, M * 1, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 0, 0, M * 2, 0).finished());
  InsertWrench(&x, id, 3, (Vector(6) << 0, 0, 0, 0, M * 6.8, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor->unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, x, diffDelta, tol);
}

// Test wrench factor for non-zero twist case, zero joint angle
TEST(WrenchFactor, NonzeroTwistCase) {
  // Create all factors
  int id = 0;
  const double M = example::inertia(3, 3);
  // gravity set to zero in this case
  auto factor = WrenchFactor(example::cost_model, example::link,
                      {WrenchKey(id, 1), WrenchKey(id, 2)},
                      0);

  Values x;
  InsertTwist(&x, id, (Vector(6) << 0, 0, 1, 0, 1, 0).finished());
  InsertTwistAccel(&x, id, (Vector(6) << 0, 0, 1, 0, 1, 0).finished());
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 4, M * -1, M * 2, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, -3, M * 0, -M, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor->unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, x, diffDelta, tol);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
