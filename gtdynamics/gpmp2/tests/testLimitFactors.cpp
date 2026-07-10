/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testLimitFactors.cpp
 * @brief test the joint angle and joint velocity limit factors.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/JointLimitFactorVector.h>
#include <gtdynamics/factors/VelocityLimitFactorVector.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <stdexcept>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Values;
using gtsam::Vector;
using gtsam::noiseModel::Isotropic;

static const gtsam::Key kKey = 0;

/* ********************** joint angle limits *************************** */

// Inside the band the cost is zero; outside it rises linearly from thresh away
// from the limit, and the Jacobian is a diagonal of -1, 0 or +1.
TEST(JointLimitFactorVector, errorAndJacobians) {
  const Vector down = (Vector(3) << -1.0, 0.0, -2.0).finished();
  const Vector up = (Vector(3) << 1.0, 13.0, 2.0).finished();
  const Vector thresh = (Vector(3) << 0.1, 0.2, 0.05).finished();
  JointLimitFactorVector factor(kKey, Isotropic::Sigma(3, 1.0), down, up,
                                thresh);
  EXPECT_LONGS_EQUAL(3, factor.dof());

  // Every joint well inside its band.
  const Vector inside = (Vector(3) << 0.0, 5.0, 0.0).finished();
  EXPECT(assert_equal(Vector(Vector::Zero(3)), factor.evaluateError(inside),
                      1e-9));

  // Below the first limit, inside the second, above the third.
  const Vector outside = (Vector(3) << -1.5, 5.0, 2.5).finished();
  const Vector expected = (Vector(3) << 0.6, 0.0, 0.55).finished();
  EXPECT(assert_equal(expected, factor.evaluateError(outside), 1e-9));

  Matrix H;
  factor.evaluateError(outside, &H);
  Matrix expected_H = Matrix::Zero(3, 3);
  expected_H(0, 0) = -1.0;
  expected_H(2, 2) = 1.0;
  EXPECT(assert_equal(expected_H, H, 1e-9));

  // The hinge is not differentiable at a knee, so probe away from one.
  Values values;
  values.insert(kKey, outside);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  Values inside_values;
  inside_values.insert(kKey, inside);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, inside_values, 1e-7, 1e-5);
}

TEST(JointLimitFactorVector, rejectsBadDimensions) {
  const Vector down = (Vector(3) << -1.0, 0.0, -2.0).finished();
  const Vector up = (Vector(3) << 1.0, 13.0, 2.0).finished();
  const Vector thresh = (Vector(3) << 0.1, 0.2, 0.05).finished();

  // A limit vector that disagrees with the noise model dimension.
  CHECK_EXCEPTION(
      JointLimitFactorVector(kKey, Isotropic::Sigma(2, 1.0), down, up, thresh),
      std::invalid_argument);

  // A threshold that leaves the joint no feasible interval.
  const Vector wide = (Vector(3) << 0.1, 0.2, 5.0).finished();
  CHECK_EXCEPTION(
      JointLimitFactorVector(kKey, Isotropic::Sigma(3, 1.0), down, up, wide),
      std::invalid_argument);

  // A configuration whose size disagrees with the limits.
  JointLimitFactorVector factor(kKey, Isotropic::Sigma(3, 1.0), down, up,
                                thresh);
  CHECK_EXCEPTION(factor.evaluateError(Vector(Vector::Zero(2))),
                  std::invalid_argument);
}

/* ******************** joint velocity limits ************************** */

// The velocity limit is the joint limit with symmetric bounds.
TEST(VelocityLimitFactorVector, errorAndJacobians) {
  const Vector limit = (Vector(3) << 1.0, 2.0, 3.0).finished();
  const Vector thresh = (Vector(3) << 0.1, 0.2, 0.3).finished();
  VelocityLimitFactorVector factor(kKey, Isotropic::Sigma(3, 1.0), limit,
                                   thresh);
  EXPECT_LONGS_EQUAL(3, factor.dof());

  const Vector inside = (Vector(3) << 0.0, 0.5, -1.0).finished();
  EXPECT(assert_equal(Vector(Vector::Zero(3)), factor.evaluateError(inside),
                      1e-9));

  // Below the first limit, inside the second, above the third.
  const Vector outside = (Vector(3) << -1.5, 0.5, 3.0).finished();
  const Vector expected = (Vector(3) << 0.6, 0.0, 0.3).finished();
  EXPECT(assert_equal(expected, factor.evaluateError(outside), 1e-9));

  Matrix H;
  factor.evaluateError(outside, &H);
  Matrix expected_H = Matrix::Zero(3, 3);
  expected_H(0, 0) = -1.0;
  expected_H(2, 2) = 1.0;
  EXPECT(assert_equal(expected_H, H, 1e-9));

  Values values;
  values.insert(kKey, outside);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  Values inside_values;
  inside_values.insert(kKey, inside);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, inside_values, 1e-7, 1e-5);
}

TEST(VelocityLimitFactorVector, rejectsBadDimensions) {
  const Vector limit = (Vector(3) << 1.0, 2.0, 3.0).finished();
  const Vector thresh = (Vector(3) << 0.1, 0.2, 0.3).finished();

  CHECK_EXCEPTION(
      VelocityLimitFactorVector(kKey, Isotropic::Sigma(2, 1.0), limit, thresh),
      std::invalid_argument);

  // A threshold wider than the limit leaves no feasible interval.
  const Vector wide = (Vector(3) << 0.1, 0.2, 4.0).finished();
  CHECK_EXCEPTION(
      VelocityLimitFactorVector(kKey, Isotropic::Sigma(3, 1.0), limit, wide),
      std::invalid_argument);

  VelocityLimitFactorVector factor(kKey, Isotropic::Sigma(3, 1.0), limit,
                                   thresh);
  CHECK_EXCEPTION(factor.evaluateError(Vector(Vector::Zero(2))),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
