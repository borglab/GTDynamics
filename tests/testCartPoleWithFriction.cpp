
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace gtsam;
using namespace gtdynamics;

// #include <gtdynamics/cmopt/CartPoleWithFriction.h>


// TEST(CartPoleWithFrictionCone, jacobian) {
//   CartPoleWithFriction cp;
//   double q = -1.0, v = 1.0, a = 1.0;
//   Matrix11 H_q, H_v, H_a;
//   Matrix11 expected_H_q, expected_H_v, expected_H_a;

//   // Test torque function jacobian
//   cp.torque(q, a, H_q, H_a);
//   auto torque_function = [&](const double q, const double a) {
//     return cp.torque(q, a);
//   };
//   expected_H_q =
//       numericalDerivative21<double, double, double, 1>(torque_function, q, a);
//   expected_H_a =
//       numericalDerivative22<double, double, double, 1>(torque_function, q, a);
//   EXPECT(assert_equal(expected_H_q, H_q));
//   EXPECT(assert_equal(expected_H_a, H_a));

//   // Test fx function jacobian
//   cp.fx(q, v, a, H_q, H_v, H_a);
//   auto fx = [&](const double q, const double v, const double a) {
//     return cp.fx(q, v, a);
//   };
//   expected_H_q = numericalDerivative31<double, double, double, double, 1>(
//       fx, q, v, a);
//   expected_H_v = numericalDerivative32<double, double, double, double, 1>(
//       fx, q, v, a);
//   expected_H_a = numericalDerivative33<double, double, double, double, 1>(
//       fx, q, v, a);
//   EXPECT(assert_equal(expected_H_q, H_q));
//   EXPECT(assert_equal(expected_H_v, H_v));
//   EXPECT(assert_equal(expected_H_a, H_a));

//   // Test fy function jacobian
//   cp.fy(q, v, a, H_q, H_v, H_a);
//   auto fy = [&](const double q, const double v, const double a) {
//     return cp.fy(q, v, a);
//   };
//   expected_H_q = numericalDerivative31<double, double, double, double, 1>(
//       fy, q, v, a);
//   expected_H_v = numericalDerivative32<double, double, double, double, 1>(
//       fy, q, v, a);
//   expected_H_a = numericalDerivative33<double, double, double, double, 1>(
//       fy, q, v, a);
//   EXPECT(assert_equal(expected_H_q, H_q));
//   EXPECT(assert_equal(expected_H_v, H_v));
//   EXPECT(assert_equal(expected_H_a, H_a));
// }

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
