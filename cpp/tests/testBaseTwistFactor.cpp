/**
 * @file  testBaseTwistFactor.cpp
 * @brief test base twist factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <BaseTwistFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test base twist factor
 */
TEST(BaseTwistFactor, error) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  Vector6 base_twist;
  base_twist << 0, 0, 0, 0, 0, 0;
  BaseTwistFactor factor(0, cost_model, base_twist);
  Vector6 conf;
  Vector6 actual_errors, expected_errors;

  conf = Vector6::Zero();
  actual_errors = factor.evaluateError(conf);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(0, conf);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test base twist factor graph optimization
 */
TEST(BaseTwistFactor, optimaization) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  Key vKey = Symbol('V', 0);
  Vector6 base_twist;
  base_twist << 0, 0, 1, 0, 0, 0;
  Vector base_twist_init = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();

  NonlinearFactorGraph graph;
  graph.add(BaseTwistFactor(vKey, cost_model, base_twist));
  Values init_values;
  init_values.insert(vKey, base_twist_init);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-6);
  EXPECT(assert_equal(base_twist, results.at<Vector>(vKey), 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
