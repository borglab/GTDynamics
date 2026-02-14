
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>

using namespace gtsam;
using namespace gtdynamics;


TEST(IECartPoleWithFrictionCone, jacobian) {
  IECartPoleWithFriction cp;
  double q = -1.0, v = 1.0, a = 1.0;
  Matrix11 H_q, H_v, H_a;
  Matrix11 expected_H_q, expected_H_v, expected_H_a;

  // Test computeFx function jacobian
  cp.computeFx(q, v, a, H_q, H_v, H_a);
  auto fx_function = [&](const double q, const double v, const double a) {
    return cp.computeFx(q, v, a);
  };
  expected_H_q = numericalDerivative31<double, double, double, double, 1>(
      fx_function, q, v, a);
  expected_H_v = numericalDerivative32<double, double, double, double, 1>(
      fx_function, q, v, a);
  expected_H_a = numericalDerivative33<double, double, double, double, 1>(
      fx_function, q, v, a);
  EXPECT(assert_equal(expected_H_q, H_q));
  EXPECT(assert_equal(expected_H_v, H_v));
  EXPECT(assert_equal(expected_H_a, H_a));

  // Test computeFy function jacobian
  cp.computeFy(q, v, a, H_q, H_v, H_a);
  auto fy_function = [&](const double q, const double v, const double a) {
    return cp.computeFy(q, v, a);
  };
  expected_H_q = numericalDerivative31<double, double, double, double, 1>(
      fy_function, q, v, a);
  expected_H_v = numericalDerivative32<double, double, double, double, 1>(
      fy_function, q, v, a);
  expected_H_a = numericalDerivative33<double, double, double, double, 1>(
      fy_function, q, v, a);
  EXPECT(assert_equal(expected_H_q, H_q));
  EXPECT(assert_equal(expected_H_v, H_v));
  EXPECT(assert_equal(expected_H_a, H_a));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
