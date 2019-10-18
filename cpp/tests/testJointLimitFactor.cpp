/**
 * @file  test_jointLimitFactor.cpp
 * @brief test joint limit factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <JointLimitFactor.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// Test joint limit factor
TEST(JointLimitFactor, error) {
  // Create all factors
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);
  // RR link example
  double lower_limit = -5.0, upper_limit = 5.0, limit_threshold = 2.0;
  JointLimitFactor factor(0, cost_model, lower_limit, upper_limit,
                          limit_threshold);
  double q;
  Vector actual_error, expected_error;
  // Make sure linearization is correct
  Values values;
  double diffDelta = 1e-7;

  // Zero errors
  q = 0;
  actual_error = factor.evaluateError(q);
  expected_error = Vector1(0);
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  values.insert(0, q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // Over lower limit
  q = -10.0;
  actual_error = factor.evaluateError(q);
  expected_error = Vector1(7.0);
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  values.update(0, q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // Over upper limit
  q = 10.0;
  actual_error = factor.evaluateError(q);
  expected_error = Vector1(7.0);
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  values.update(0, q);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

TEST(JointLimitFactor, optimization) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model =
      noiseModel::Isotropic::Sigma(1, 1000);

  Key qkey = Symbol('x', 0);
  // RR link example
  double lower_limit = -5.0, upper_limit = 5.0, limit_threshold = 1;

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactor(qkey, cost_model, lower_limit, upper_limit,
                             limit_threshold));

  double q = -6;
  graph.add(PriorFactor<double>(qkey, q, prior_model));
  Values init_value;
  init_value.insert(qkey, q);

  LevenbergMarquardtParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  LevenbergMarquardtOptimizer optimizer(graph, init_value, parameters);

  optimizer.optimize();
  Values result = optimizer.values();

  double q_limit = -4;
  EXPECT(assert_equal(q_limit, result.at<double>(qkey), 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}