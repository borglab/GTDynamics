/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testWrenchEquivalenceFactor.cpp
 * @brief Test wrench factor.
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

#include <cmath>
#include <iostream>

#include "gtdynamics/factors/WrenchEquivalenceFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Vector6, gtsam::Vector3, gtsam::Vector, gtsam::Pose3, gtsam::Rot3,
    gtsam::Point3, gtsam::Values;

namespace example {
// Noise model.
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key twist_key = gtsam::Symbol('V', 1),
           twist_accel_key = gtsam::Symbol('T', 1),
           wrench_j_key = gtsam::Symbol('W', 1),
           wrench_k_key = gtsam::Symbol('W', 2), qKey = gtsam::Symbol('q', 1),
           pKey = gtsam::Symbol('p', 1);
}  // namespace example

/**
 * Test wrench equivalence factor
 */
TEST(WrenchEquivalenceFactor, error_1) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(kMj, screw_axis);

  WrenchEquivalenceFactor factor(example::wrench_j_key,
  example::wrench_k_key, example::qKey, example::cost_model, joint);

  double q = 0;
  Vector wrench_j, wrench_k;
  wrench_j = (Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 19.6, 0, -9.8, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(wrench_j, wrench_k, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench equivalence factor
 */
TEST(WrenchEquivalenceFactor, error_2) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(kMj, screw_axis);

  WrenchEquivalenceFactor factor(example::wrench_j_key,
      example::wrench_k_key, example::qKey, example::cost_model, joint);
  double q = -M_PI_2;
  Vector wrench_j, wrench_k;
  wrench_j = (Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 9.8, 9.8, 0, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(wrench_j, wrench_k, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench equivalence factor
 */
TEST(WrenchEquivalenceFactor, error_3) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(0, 0, -2));
  Vector6 screw_axis;
  screw_axis << 1, 0, 0, 0, -1, 0;

  auto joint = make_joint(kMj, screw_axis);

  WrenchEquivalenceFactor factor(example::wrench_j_key, example::wrench_k_key,
                                 example::qKey, example::cost_model, joint);
  double q = 0;
  Vector wrench_j, wrench_k;
  wrench_j = (Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  wrench_k = (Vector(6) << -1, 0, 0, 0, 0, 0).finished();
  Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(wrench_j, wrench_k, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
