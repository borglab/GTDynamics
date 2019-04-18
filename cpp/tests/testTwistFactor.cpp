/**
 * @file  testTwistFactor.cpp
 * @brief test twist factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <DHLink.h>
#include <TwistFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// R link example
DH_Link dh_r = DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Vector3(0, 0, 0),
                       -180, 10, 180);
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);
Key twist_i_key = Symbol('V', 1), twist_j_key = Symbol('V', 2),
    qKey = Symbol('q', 0), qVelKey = Symbol('j', 0);
}  // namespace example

/**
 * Test twist factor for stationary case
 */
TEST(TwistFactor, error) {
  // Create all factors
  Pose3 jMi = Pose3(Rot3(), Point3(-1, 0, 0));
  Vector6 screw_axis = (Vector(6) << 0, 0, 1, 0, 1, 0).finished();

  TwistFactor factor(example::twist_i_key, example::twist_j_key, example::qKey,
                     example::qVelKey, example::cost_model, jMi, screw_axis);
  double q = M_PI / 4, qVel = 10;
  Vector6 twist_i, twist_j;
  twist_i = (Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_j = (Vector(6) << 0, 0, 20, 7.07106781, 27.0710678, 0).finished();
  Vector actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, expected_H1, expected_H2,
      expected_H3, expected_H4;

  actual_errors = factor.evaluateError(twist_i, twist_j, q, qVel, actual_H1,
                                       actual_H2, actual_H3, actual_H4);
  expected_errors = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&TwistFactor::evaluateError, factor, _1, twist_j, q, qVel,
                      boost::none, boost::none, boost::none, boost::none)),
      twist_i, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&TwistFactor::evaluateError, factor, twist_i, _1, q, qVel,
                      boost::none, boost::none, boost::none, boost::none)),
      twist_j, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const double &)>(boost::bind(
          &TwistFactor::evaluateError, factor, twist_i, twist_j, _1, qVel,
          boost::none, boost::none, boost::none, boost::none)),
      q, 1e-6);
  expected_H4 = numericalDerivative11(
      boost::function<Vector(const double &)>(
          boost::bind(&TwistFactor::evaluateError, factor, twist_i, twist_j, q,
                      _1, boost::none, boost::none, boost::none, boost::none)),
      qVel, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
