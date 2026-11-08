
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/kinematics/PoseFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtdynamics/scenarios/CartPoleWithFrictionRetractor.h>

using namespace gtsam;
using namespace gtdynamics;


TEST(IECartPoleWithFrictionCone, jacobian) {
  IECartPoleWithFriction cp;
  double q = -1.0, v = 1.0, a = 1.0;
  Matrix11 H_q, H_v, H_a;
  Matrix11 expected_H_q, expected_H_v, expected_H_a;

  // Torque derivatives still belong to the ambient constraint definition.
  cp.computeTau(q, a, H_q, H_a);
  auto torque = [&](double angle, double acceleration) {
    return cp.computeTau(angle, acceleration);
  };
  EXPECT(assert_equal(numericalDerivative21<double, double, double, 1>(
                          torque, q, a), H_q));
  EXPECT(assert_equal(numericalDerivative22<double, double, double, 1>(
                          torque, q, a), H_a));

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

// Both friction faces remain relevant under explicit type/point construction.
TEST(IECartPoleWithFrictionCone, RetractionToBothFrictionFaces) {
  IECartPoleWithFriction cp;
  const int k = 0;
  const double q = M_PI_2, v = 0;
  const auto values = cp.computeValues(k, q, v, 0);
  IEConstraintManifold::Params params;
  params.retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<CartPoleWithFrictionRetractor>(cp));
  const auto type = std::make_shared<IEConstraintManifold>(
      cp.eConstraints(k), cp.iConstraints(k), values, params);
  const auto point = type->point(values);
  const double limit = cp.friction_coefficient_ *
      (cp.cart_mass_ + cp.mass_) * cp.gravity_ / (cp.mass_ * cp.length_);
  for (double acceleration : {-30., 20.}) {
    const auto proposed = cp.computeValues(k, q, v, acceleration);
    const auto result = point.retract(values.localCoordinates(proposed));
    const double expectedAcceleration = acceleration < 0 ? -limit : limit;
    const auto expected = cp.computeValues(k, q, v, expectedAcceleration);
    EXPECT(assert_equal(expected, result.values(), 1e-8));
    EXPECT_DOUBLES_EQUAL(0, result.evaluateEqualityViolation(), 1e-8);
    EXPECT_DOUBLES_EQUAL(0, result.evaluateInequalityViolation(), 1e-8);
    EXPECT(result.activeIndices() == IndexSet{acceleration < 0 ? size_t(0)
                                                            : size_t(1)});
    EXPECT(result.manifold() == type);
    EXPECT(assert_equal(values, point.values()));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
