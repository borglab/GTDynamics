/**
 * @file  test_jointLimitFactor.cpp
 * @brief test joint limit factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <JointLimitFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test joint limit factor
 */
TEST(JointLimitFactor, error) {
  // Create all factors
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);
  // RR link example
  double lower_limit = -5.0, upper_limit = 5.0, miu = 1e-10;
  JointLimitFactor factor(0, cost_model, lower_limit, upper_limit, miu);
  double q;
  Vector1 actual_error, expected_error;
  Matrix actual_H, expected_H;

  // Zero errors
  q = 0;
  actual_error = factor.evaluateError(q, actual_H);
  expected_error << 0;
  expected_H = numericalDerivative11(
      boost::function<Vector(const double &)>(boost::bind(
          &JointLimitFactor::evaluateError, factor, _1, boost::none)),
      q, 1e-6);
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));

  // Non-zero error but in limit
  q = 3;
  actual_error = factor.evaluateError(q, actual_H);
  expected_error << 9.16291e-10;
  expected_H = numericalDerivative11(
      boost::function<Vector(const double &)>(boost::bind(
          &JointLimitFactor::evaluateError, factor, _1, boost::none)),
      q, 1e-6);
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));

  // Over lower limit
  q = -10.0;
  actual_error = factor.evaluateError(q, actual_H);
  expected_error << std::numeric_limits<double>::infinity();
  expected_H = (gtsam::Matrix(1, 1) << 0).finished();
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));

  // Over upper limit
  q = 10.0;
  actual_error = factor.evaluateError(q, actual_H);
  expected_error << std::numeric_limits<double>::infinity();
  expected_H = (gtsam::Matrix(1, 1) << 0).finished();
  EXPECT(assert_equal(expected_error, actual_error, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

TEST(JointLimitFactor, optimization) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model =
      noiseModel::Isotropic::Sigma(1, 1000);

  Key qkey = Symbol('x', 0);
  // RR link example
  double lower_limit = -5.0, upper_limit = 5.0, miu = 1e-10;

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactor(qkey, cost_model, lower_limit, upper_limit, miu));

  double q = -4;
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