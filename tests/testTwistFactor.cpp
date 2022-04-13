/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTwistFactor.cpp
 * @brief Test twist factor.
 * @author Frank Dellaert and Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/TwistFactor.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
gtsam::Key twist_p_key = TwistKey(1), twist_c_key = TwistKey(2),
           qKey = JointAngleKey(1), qVelKey = JointVelKey(1);
}  // namespace example

// Test twist factor for stationary case
TEST(TwistFactor, error) {
  // Create all factors
  gtsam::Pose3 cMp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(cMp, screw_axis);

  auto factor = TwistFactor(example::cost_model, joint, 0);
  double q = M_PI / 4, qVel = 10;
  gtsam::Vector twist_p, twist_c;
  twist_p = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_c =
      (gtsam::Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  gtsam::Values values;
  values.insert(example::qKey, q);
  values.insert(example::qVelKey, qVel);
  values.insert(example::twist_p_key, twist_p);
  values.insert(example::twist_c_key, twist_c);

  gtsam::Vector6 actual_errors, expected_errors;
  actual_errors = factor->unwhitenedError(values);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct

  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
