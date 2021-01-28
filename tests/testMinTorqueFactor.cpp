/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testMinTorqueFactor.cpp
 * @brief Test min torque factor.
 * @author Alejandro Escontrela
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

#include "gtdynamics/factors/MinTorqueFactor.h"

using namespace gtdynamics;
using gtsam::assert_equal;

namespace example {

// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
gtsam::Key torque_key = gtsam::Symbol('t', 1);
}  // namespace example

// Test Torque factor for stationary case
TEST(MinTorqueFactor, error) {
  MinTorqueFactor factor(example::torque_key, example::cost_model);

  EXPECT(assert_equal((gtsam::Vector(1) << 20).finished(),
                      factor.evaluateError(20), 1e-6));
  // Make sure linearization is correct
  gtsam::Values values;
  values.insert(example::torque_key, 20.0);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a joint torque to ensure zero torque
 * reached.
 **/
TEST(MinTorqueFactor, optimization) {
  MinTorqueFactor factor(example::torque_key, example::cost_model);

  // Initial torque.
  double torque_init = 1e8;

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(example::torque_key, torque_init);

  gtsam::GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-14);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();

  gtsam::Values results = optimizer.values();
  double torque_optimized = results.atDouble(example::torque_key);

  EXPECT(assert_equal(0.0, torque_optimized, 1e-3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
