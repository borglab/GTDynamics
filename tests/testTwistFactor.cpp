/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTwistFactor.cpp
 * @brief Test twist factor.
 * @Author: Frank Dellaert and Mandy Xie
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

#include "gtdynamics/factors/TwistFactor.h"

using namespace gtdynamics; 
using gtsam::assert_equal;

namespace example {
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key twist_p_key = gtsam::Symbol('V', 1),
           twist_c_key = gtsam::Symbol('V', 2), qKey = gtsam::Symbol('q', 0),
           qVelKey = gtsam::Symbol('j', 0);
}  // namespace example

// Test twist factor for stationary case
TEST(TwistFactor, error) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  TwistFactor factor(example::twist_p_key, example::twist_c_key,
                                 example::qKey, example::qVelKey,
                                 example::cost_model, cMp, screw_axis);
  double q = M_PI / 4, qVel = 10;
  gtsam::Vector twist_p, twist_c;
  twist_p = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_c =
      (gtsam::Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  gtsam::Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(twist_p, twist_c, q, qVel);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
