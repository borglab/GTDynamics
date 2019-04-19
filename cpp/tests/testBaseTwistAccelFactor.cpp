/**
 * @file  testBaseTwistAccelFactor.cpp
 * @brief test base acceleration factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <BaseTwistAccelFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test base acceleration factor
 */
TEST(BaseTwistAccelFactor, error) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  Vector6 base_acceleration;
  base_acceleration << 0, 0, 0, 0, 9.8, 0;
  BaseTwistAccelFactor factor(LabeledSymbol('G', '0', 0), cost_model,
                              base_acceleration);
  Vector6 conf;
  Vector6 actual_errors, expected_errors;
  Matrix actual_H, expected_H;

  conf = Vector6::Zero();
  actual_errors = factor.evaluateError(conf, actual_H);
  expected_errors << 0, 0, 0, 0, -9.8, 0;
  expected_H = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(boost::bind(
          &BaseTwistAccelFactor::evaluateError, factor, _1, boost::none)),
      conf, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

/**
 * Test base acceleration factor graph optimization
 */
TEST(BaseTwistAccelFactor, optimaization) {
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  Key aKey = LabeledSymbol('G', '0', 0);
  Vector6 base_acceleration;
  base_acceleration << 0, 0, 0, 0, 9.8, 0;
  Vector base_acceleration_init = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();

  NonlinearFactorGraph graph;
  graph.add(BaseTwistAccelFactor(aKey, cost_model, base_acceleration));
  Values init_values;
  init_values.insert(aKey, base_acceleration_init);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-6);
  EXPECT(assert_equal(base_acceleration, results.at<Vector>(aKey), 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
