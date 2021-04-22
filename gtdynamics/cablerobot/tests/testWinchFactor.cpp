/**
 * @file  testWinchFactor.cpp
 * @brief test winch factor
 * @author Gerry Chen
 */

#include <gtdynamics/cablerobot/factors/WinchFactor.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>
#include <array>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;

/**
 * Test winch factor
 */
TEST(WinchFactor, error) {
  // noise model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(1, 1.0);
  WinchParams params(0.12,   // radius
                     1.23,   // inerita
                     0.34,   // static friction
                     0.56);  // viscous friction

  Symbol torqueKey = symbol('q', 0);
  Symbol tensionKey = symbol('t', 0);
  Symbol jointVelKey = symbol('v', 0);
  Symbol jointAccKey = symbol('a', 0);
  WinchFactor factor(torqueKey, tensionKey, jointVelKey, jointAccKey,
                     cost_model, params);

  //                                                    q   t   v   a
  EXPECT(assert_equal(Vector1(0), factor.evaluateError( 0,  0,  0,  0)));
  EXPECT(assert_equal(Vector1(0), factor.evaluateError(.12, 1,  0,  0)));
  // inertia cancels out tension exactly
  EXPECT(assert_equal(Vector1(0),
                      factor.evaluateError(0, 1, 0, 0.12 * 0.12 / 1.23)));
  // viscous friction
  EXPECT(assert_equal(
      Vector1(0),  // forward velocity -> extending cable -> negative torque
      factor.evaluateError(-0.4621171573 * 0.34 - 0.01 / .12 * 0.56, 0, .01,
                           0)));
  EXPECT(assert_equal(Vector1(0),
                      factor.evaluateError(-0.34 - 1 / 0.12 * 0.56, 0, 1, 0)));
  EXPECT(assert_equal(Vector1(0),
                      factor.evaluateError(+0.34 + 1 / 0.12 * 0.56, 0, -1, 0)));
  // viscous friction with tension
  EXPECT(assert_equal(
      Vector1(0),
      factor.evaluateError(0.12 + 0.34 + 1 / 0.12 * 0.56, 1, -1, 0)));

  Values values;
  values.insertDouble(torqueKey, 1);
  values.insertDouble(tensionKey, 2);
  values.insertDouble(jointVelKey, .03);
  values.insertDouble(jointAccKey, 4);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
