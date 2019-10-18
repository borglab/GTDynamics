/**
 * @file  testJointLimitVectorFactor.cpp
 * @brief test joint limit vector factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <JointLimitVectorFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// Test joint limit vector factor
TEST(JointLimitVectorFactor, error) {
  // Create all factors
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(2, 1.0);
  // RR link example
  Vector2 lower_limits(-5.0, -10.0), upper_limits(5.0, 10.0);
  Vector2 limit_thresholds(2.0, 2.0);
  JointLimitVectorFactor factor(0, cost_model, lower_limits, upper_limits,
                                limit_thresholds);
  Vector conf;
  Vector actual_errors, expected_errors;
  // Make sure linearization is correct
  Values values;
  double diffDelta = 1e-7;

  // Zero errors
  conf = Vector2::Zero();
  actual_errors = factor.evaluateError(conf);
  expected_errors = Vector2::Zero();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  values.insert(0, conf);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
 
  // Over lower limit
  conf = Vector2(-10.0, -10.0);
  actual_errors = factor.evaluateError(conf);
  expected_errors = Vector2(7.0, 2.0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  values.update(0, conf);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // Over upper limit
  conf = Vector2(10.0, 10.0);
  actual_errors = factor.evaluateError(conf);
  expected_errors = Vector2(7.0, 2.0);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  values.update(0, conf);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test joint limit vector factor graph optimization
TEST(JointLimitVectorFactor, optimaization) {
  // settings
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(2, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model =
      noiseModel::Isotropic::Sigma(2, 1000);
  Key qkey = Symbol('x', 0);
  Vector2 dlimit(-5.0, -10.0), ulimit(5, 10.0);
  Vector2 thresh(2.0, 2.0);

  Vector conf;
  conf = (Vector(2) << 10.0, 10.0).finished();

  NonlinearFactorGraph graph;
  graph.add(JointLimitVectorFactor(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  Vector conf_limit = (Vector(2) << 3.0, 8.0).finished();
  EXPECT(assert_equal(conf_limit, results.at<Vector>(qkey), 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
