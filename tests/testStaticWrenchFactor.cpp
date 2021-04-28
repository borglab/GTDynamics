/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testStaticWrenchFactor.cpp
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

#include "gtdynamics/statics/StaticWrenchFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using internal::PoseKey;
using internal::TwistAccelKey;
using internal::TwistKey;
using internal::WrenchKey;
using namespace gtsam;

namespace example {

// R link example
using simple_urdf::robot;

const Matrix6 inertia = robot.links()[0]->inertiaMatrix();
const Vector3 gravity(0, -9.8, 0);

noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);

}  // namespace example

constexpr double diffDelta = 1e-4;
constexpr double tol = 1e-5;

// Test statics when compensating exactly for gravity
TEST(StaticWrenchFactor, GravityCompensation) {
  // Create all factors
  int id = 0;
  const double M = example::inertia(3, 3);
  StaticWrenchFactor factor(
      {WrenchKey(id, 1), WrenchKey(id, 2), WrenchKey(id, 3)}, PoseKey(id),
      example::cost_model, example::inertia, example::gravity);
  Values x;
  InsertWrench(&x, id, 1, (Vector(6) << 0, 0, 0, 0, M * 1, 0).finished());
  InsertWrench(&x, id, 2, (Vector(6) << 0, 0, 0, 0, M * 2, 0).finished());
  InsertWrench(&x, id, 3, (Vector(6) << 0, 0, 0, 0, M * 6.8, 0).finished());
  InsertPose(&x, id, Pose3(Rot3(), Point3(1, 0, 0)));

  Vector6 actual_errors = factor.unwhitenedError(x);
  Vector6 expected_errors = Z_6x1;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, x, diffDelta, tol);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
