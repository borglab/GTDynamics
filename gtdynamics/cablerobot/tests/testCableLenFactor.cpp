/**
 * @file  testCableLenFactor.cpp
 * @brief test cable vel factor
 * @Author: Frank Dellaert and Gerry Chen
 */

#include "factors/CableLenFactor.h"

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gtdynamics::cablerobot;

/**
 * Test cable factor
 */
TEST(CableLenFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol points[2] = {symbol('p', 0), symbol('p', 1)};
  Symbol l = symbol('t', 0);
  CableLenFactor<Point3> factor(l, points[0], points[1], cost_model);

  double conf_l = 1;
  Point3 conf_points[2] = {Point3(0, 0, 0), Point3(1, 0, 0)};
  Vector1 expected_errors { 0 };

  Vector1 actual_errors =
      factor.evaluateError(conf_l, conf_points[0], conf_points[1]);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  Values values;
  values.insertDouble(l, conf_l);
  values.insert(points[0], conf_points[0]);
  values.insert(points[1], conf_points[1]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(points[1], Point3(3, 5, 7));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

/**
 * Test cable factor, in Point2
 */
TEST(CableLenFactor, errorPoint2) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol points[2] = {symbol('p', 0), symbol('p', 1)};
  Symbol l = symbol('t', 0);
  CableLenFactor<Point2> factor(l, points[0], points[1],
                                cost_model);

  double conf_l = 2 * 1.4142135624;
  Point2 conf_points[2] = {Point2(5, 5), Point2(3, 7)};
  Vector1 expected_errors { 0 };

  Vector1 actual_errors =
      factor.evaluateError(conf_l, conf_points[0], conf_points[1]);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  Values values;
  values.insertDouble(l, conf_l);
  values.insert(points[0], conf_points[0]);
  values.insert(points[1], conf_points[1]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(points[1], Point2(3, 7));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}