/**
 * @file  testCableTensionFactor.cpp
 * @brief test cable factor
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "factors/CableTensionFactor.h"

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
TEST(CableTensionFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);

  Symbol points[2] = {symbol('p', 0), symbol('p', 1)};
  Symbol forces[2] = {symbol('f', 0), symbol('f', 1)};
  Symbol tension = symbol('t', 0);
  CableTensionFactor factor(tension, points[0], points[1], forces[0], forces[1], cost_model);

  double conf_tension = 2;
  Point3 conf_points[2] = {Point3(0, 0, 0), Point3(1, 0, 0)};
  Vector conf_forces[2] = {Vector3(1, 0, 0), Vector3(-1, 0, 0)};
  Vector6 expected_errors;
  expected_errors << 1, 0, 0, -1, 0, 0;  // expect forces of 2i and -2i

  Vector6 actual_errors = factor.evaluateError(
    conf_tension, conf_points[0], conf_points[1], conf_forces[0], conf_forces[1]);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-4));

  Values values;
  values.insertDouble(tension, conf_tension);
  values.insert(points[0], conf_points[0]);
  values.insert(points[1], conf_points[1]);
  values.insert(forces[0], conf_forces[0]);
  values.insert(forces[1], conf_forces[1]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);

  values.update(points[1], Point3(3, 5, 7));
  values.update(forces[1], static_cast<gtsam::Vector>(Vector3(-13, 17, 19)));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}