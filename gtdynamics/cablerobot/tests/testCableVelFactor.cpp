/**
 * @file  testCableVelFactor.cpp
 * @brief test cable vel factor
 * @Author: Frank Dellaert and Gerry Chen
 */

#include "factors/CableVelFactor.h"

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
TEST(CableVelFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol points[2] = {symbol('p', 0), symbol('p', 1)};
  Symbol vels[2] = {symbol('f', 0), symbol('f', 1)};
  Symbol v = symbol('t', 0);
  CableVelFactor<Point3> factor(v, points[0], points[1], vels[0], vels[1],
                                cost_model);

  double conf_v = -2;
  Point3 conf_points[2] = {Point3(0, 0, 0), Point3(1, 0, 0)};
  Vector conf_vels[2] = {Vector3(1, 0, 0), Vector3(-1, 0, 0)};
  Vector1 expected_errors { 0 };

  Vector1 actual_errors = factor.evaluateError(
    conf_v, conf_points[0], conf_points[1], conf_vels[0], conf_vels[1]);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  Values values;
  values.insertDouble(v, conf_v);
  values.insert(points[0], conf_points[0]);
  values.insert(points[1], conf_points[1]);
  values.insert(vels[0], conf_vels[0]);
  values.insert(vels[1], conf_vels[1]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(points[1], Point3(3, 5, 7));
  values.update(vels[1], static_cast<gtsam::Vector>(Vector3(-13, 17, 19)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

/**
 * Test cable factor, in Point2
 */
TEST(CableVelFactor, errorPoint2) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);

  Symbol points[2] = {symbol('p', 0), symbol('p', 1)};
  Symbol vels[2] = {symbol('f', 0), symbol('f', 1)};
  Symbol v = symbol('t', 0);
  CableVelFactor<Point2> factor(v, points[0], points[1], vels[0], vels[1],
                                cost_model);

  double conf_v = -5;
  Point2 conf_points[2] = {Point2(5, 5), Point2(3, 7)};
  Vector conf_vels[2] = {Vector2(0, 0), Vector2(5 * 1.4142135624, 0)};
  Vector1 expected_errors { 0 };

  Vector1 actual_errors = factor.evaluateError(
    conf_v, conf_points[0], conf_points[1], conf_vels[0], conf_vels[1]);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  Values values;
  values.insertDouble(v, conf_v);
  values.insert(points[0], conf_points[0]);
  values.insert(points[1], conf_points[1]);
  values.insert(vels[0], conf_vels[0]);
  values.insert(vels[1], conf_vels[1]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(points[1], Point2(3, 7));
  values.update(vels[1], static_cast<gtsam::Vector>(Vector2(-13, 19)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}