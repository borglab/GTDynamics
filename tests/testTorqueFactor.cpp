/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTorqueFactor.cpp
 * @brief Test torque factor.
 * @author Frank Dellaert and Mandy Xie
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

#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/values.h"
#include "make_joint.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Pose3, gtsam::Vector6, gtsam::Vector3, gtsam::Rot3, gtsam::Point3;

// Test Torque factor for stationary case
TEST(TorqueFactor, error) {
  Pose3 kMj = Pose3(Rot3(), Point3(0, 0, -2));  // doesn't matter
  gtsam::Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;

  auto joint = make_joint(kMj, screw_axis);

  // Create factor.
  auto cost_model = gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
  auto factor = TorqueFactor(cost_model, joint, 777);

  // Check keys.
  const DynamicsSymbol wrench_key = internal::WrenchKey(2, 1, 777),
                       torque_key = internal::TorqueKey(1, 777);
  EXPECT(assert_equal(wrench_key, factor.keys()[0]));
  EXPECT(assert_equal(torque_key, factor.keys()[1]));

  // Check evaluateError.
  double torque = 20;
  gtsam::Vector wrench = (gtsam::Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  gtsam::Values values;
  values.insert(wrench_key, wrench);
  values.insert(torque_key, torque);
  gtsam::Vector1 actual_errors = factor.unwhitenedError(values);
  gtsam::Vector1 expected_errors(0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct.
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-7);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
