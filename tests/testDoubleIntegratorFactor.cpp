/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDoubleIntegratorFactor.cpp
 * @brief Test DoubleIntegratorFactor.
 * @author Harrison Jenkins
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/DoubleIntegratorFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <stdexcept>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector;

namespace {

/// The five variables of one step, under q1, v1, q2, v2, u.
Values stepValues(const Vector &q1, const Vector &v1, const Vector &q2,
                  const Vector &v2, const Vector &u) {
  Values values;
  values.insert(Symbol('q', 1), q1);
  values.insert(Symbol('v', 1), v1);
  values.insert(Symbol('q', 2), q2);
  values.insert(Symbol('v', 2), v2);
  values.insert(Symbol('u', 1), u);
  return values;
}

/// A factor on the variables of stepValues, with n joints.
DoubleIntegratorFactor makeFactor(double dt, int n, double sigma = 0.01) {
  return DoubleIntegratorFactor(
      Symbol('q', 1), Symbol('v', 1), Symbol('q', 2), Symbol('v', 2),
      Symbol('u', 1), dt, gtsam::noiseModel::Isotropic::Sigma(2 * n, sigma));
}

}  // namespace

/// States that follow constant-acceleration kinematics have zero error.
TEST(DoubleIntegratorFactor, ZeroErrorOnConstantAcceleration) {
  const double dt = 0.6;
  const Vector q1 = (Vector(3) << 0.1, -0.4, 0.8).finished();
  const Vector v1 = (Vector(3) << 0.2, 0.0, -0.3).finished();
  const Vector u = (Vector(3) << 0.5, -1.0, 0.25).finished();
  const Vector q2 = q1 + dt * v1 + 0.5 * dt * dt * u;
  const Vector v2 = v1 + dt * u;

  const Vector error =
      makeFactor(dt, 3).unwhitenedError(stepValues(q1, v1, q2, v2, u));
  EXPECT_LONGS_EQUAL(6, error.size());
  EXPECT(assert_equal(Vector::Zero(6), error, 1e-14));
}

/// For arbitrary states the error is x2 - Fx x1 - Fu u, written out.
TEST(DoubleIntegratorFactor, ErrorForArbitraryStates) {
  const double dt = 0.45;
  const Vector q1 = (Vector(2) << 0.3, 0.7).finished();
  const Vector v1 = (Vector(2) << -0.2, 0.5).finished();
  const Vector q2 = (Vector(2) << 0.9, -0.2).finished();
  const Vector v2 = (Vector(2) << 0.1, 0.4).finished();
  const Vector u = (Vector(2) << 1.1, -0.6).finished();

  Vector expected(4);
  expected << q2 - q1 - dt * v1 - 0.5 * dt * dt * u, v2 - v1 - dt * u;
  const Vector error =
      makeFactor(dt, 2).unwhitenedError(stepValues(q1, v1, q2, v2, u));
  EXPECT(assert_equal(expected, error, 1e-14));
}

/// Analytic Jacobians match numerical differentiation.
TEST(DoubleIntegratorFactor, Jacobians) {
  const auto factor = makeFactor(0.8, 3);
  const Values values = stepValues((Vector(3) << 0.3, 0.7, 0.1).finished(),
                                   (Vector(3) << -0.2, 0.5, 0.2).finished(),
                                   (Vector(3) << 0.9, -0.2, 0.3).finished(),
                                   (Vector(3) << 0.1, 0.4, -0.5).finished(),
                                   (Vector(3) << 1.1, -0.6, 0.4).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/// Equality depends on the keys and on dt.
TEST(DoubleIntegratorFactor, Equals) {
  const auto factor = makeFactor(0.5, 2);
  EXPECT(factor.equals(makeFactor(0.5, 2)));
  EXPECT(!factor.equals(makeFactor(0.4, 2)));
  EXPECT_DOUBLES_EQUAL(0.5, factor.dt(), 0.0);
}

/// A non-positive dt or mismatched variable sizes are rejected.
TEST(DoubleIntegratorFactor, RejectsBadArguments) {
  CHECK_EXCEPTION(makeFactor(0.0, 2), std::invalid_argument);

  const auto factor = makeFactor(0.5, 2);
  const Vector zero2 = Vector::Zero(2);
  CHECK_EXCEPTION(factor.unwhitenedError(
                      stepValues(zero2, zero2, zero2, zero2, Vector::Zero(3))),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
