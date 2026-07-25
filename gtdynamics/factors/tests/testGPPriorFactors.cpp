/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testGPPriorFactors.cpp
 * @brief test Gaussian process prior factors, linear and Lie group versions.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/GPLiePriorFactor.h>
#include <gtdynamics/factors/GPLinearPriorFactor.h>
#include <gtdynamics/factors/GPPose3PriorFactor.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <stdexcept>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::noiseModel::Isotropic;

static const double kDeltaT = 0.4;
static const gtsam::Key kP1 = 0, kV1 = 1, kP2 = 2, kV2 = 3;

// The linear prior error is Phi(dt) * x1 - x2, so a constant velocity state
// pair, where q2 = q1 + dt * v1 and v2 == v1, lies exactly on the prior.
TEST(GPLinearPrior, errorOnConstantVelocity) {
  const size_t dof = 3;
  auto QcModel = Isotropic::Sigma(dof, 1.0);
  GPLinearPrior factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Vector q1 = (Vector(3) << 1.0, -2.0, 0.5).finished();
  const Vector v1 = (Vector(3) << 0.3, 0.7, -0.2).finished();
  const Vector q2 = q1 + kDeltaT * v1;
  const Vector v2 = v1;

  EXPECT(assert_equal(Vector::Zero(2 * dof),
                      factor.evaluateError(q1, v1, q2, v2), 1e-9));
}

// Away from a constant velocity pair the error is the shortfall in each block.
TEST(GPLinearPrior, errorAndJacobians) {
  const size_t dof = 2;
  auto QcModel = Isotropic::Sigma(dof, 0.5);
  GPLinearPrior factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Vector q1 = (Vector(2) << 1.0, 2.0).finished();
  const Vector v1 = (Vector(2) << 0.1, -0.4).finished();
  const Vector q2 = (Vector(2) << 1.5, 1.5).finished();
  const Vector v2 = (Vector(2) << 0.2, -0.3).finished();

  const Vector expected =
      (Vector(4) << q1 + kDeltaT * v1 - q2, v1 - v2).finished();
  EXPECT(assert_equal(expected, factor.evaluateError(q1, v1, q2, v2), 1e-9));

  Values values;
  values.insert(kP1, q1);
  values.insert(kV1, v1);
  values.insert(kP2, q2);
  values.insert(kV2, v2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// The Lie prior error is [Logmap(Inv(p1) * p2) - dt * v1; v2 - v1], so a state
// pair swept out by a constant body twist lies exactly on the prior. Note the
// sign convention differs from GPLinearPrior, whose error is Phi * x1 - x2.
TEST(GPLiePrior, rot3ErrorOnConstantTwist) {
  const size_t dof = 3;
  auto QcModel = Isotropic::Sigma(dof, 1.0);
  GPLiePrior<Rot3> factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Vector v1 = (Vector(3) << 0.4, -0.1, 0.6).finished();
  const Rot3 R2 = R1 * Rot3::Expmap(kDeltaT * v1);
  const Vector v2 = v1;

  EXPECT(assert_equal(Vector::Zero(2 * dof),
                      factor.evaluateError(R1, v1, R2, v2), 1e-9));
}

// Rot3 exercises the template on a group whose Expmap is not the identity map.
TEST(GPLiePrior, rot3ErrorAndJacobians) {
  const size_t dof = 3;
  auto QcModel = Isotropic::Sigma(dof, 1.0);
  GPLiePrior<Rot3> factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Rot3 R2 = Rot3::RzRyRx(-0.3, 0.15, 0.05);
  const Vector v1 = (Vector(3) << 0.4, -0.1, 0.6).finished();
  const Vector v2 = (Vector(3) << 0.2, 0.3, -0.5).finished();

  const Vector r = Rot3::Logmap(R1.inverse() * R2);
  const Vector expected = (Vector(6) << r - kDeltaT * v1, v2 - v1).finished();
  EXPECT(assert_equal(expected, factor.evaluateError(R1, v1, R2, v2), 1e-9));

  Values values;
  values.insert(kP1, R1);
  values.insert(kV1, v1);
  values.insert(kP2, R2);
  values.insert(kV2, v2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// GPPose3Prior is the Pose3 instantiation of GPLiePrior, the 3D case we use.
TEST(GPPose3Prior, errorOnConstantTwist) {
  auto QcModel = Isotropic::Sigma(6, 1.0);
  GPPose3Prior factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Pose3 p1(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const Vector v1 = (Vector(6) << 0.1, -0.2, 0.3, 0.4, 0.5, -0.6).finished();
  const Pose3 p2 = p1 * Pose3::Expmap(kDeltaT * v1);
  const Vector v2 = v1;

  EXPECT(
      assert_equal(Vector::Zero(12), factor.evaluateError(p1, v1, p2, v2), 1e-9));
}

TEST(GPPose3Prior, errorAndJacobians) {
  auto QcModel = Isotropic::Sigma(6, 1.0);
  GPPose3Prior factor(kP1, kV1, kP2, kV2, kDeltaT, QcModel);

  const Pose3 p1(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const Pose3 p2(Rot3::RzRyRx(-0.2, 0.1, 0.4), Point3(1.4, 1.7, 3.2));
  const Vector v1 = (Vector(6) << 0.1, -0.2, 0.3, 0.4, 0.5, -0.6).finished();
  const Vector v2 = (Vector(6) << 0.0, 0.1, -0.1, 0.2, -0.3, 0.4).finished();

  const Vector r = Pose3::Logmap(p1.inverse() * p2);
  const Vector expected = (Vector(12) << r - kDeltaT * v1, v2 - v1).finished();
  EXPECT(assert_equal(expected, factor.evaluateError(p1, v1, p2, v2), 1e-9));

  Values values;
  values.insert(kP1, p1);
  values.insert(kV1, v1);
  values.insert(kP2, p2);
  values.insert(kV2, v2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// A Qc whose dimension disagrees with the group would silently mis-size the
// error, so the constructor rejects it. Pose3 demands a 6 dimensional Qc.
TEST(GPPose3Prior, rejectsWrongQcDimension) {
  auto Qc3d = Isotropic::Sigma(3, 1.0);
  CHECK_EXCEPTION(GPPose3Prior(kP1, kV1, kP2, kV2, kDeltaT, Qc3d),
                  std::invalid_argument);

  auto Qc6d = Isotropic::Sigma(6, 1.0);
  CHECK_EXCEPTION(GPLiePrior<Rot3>(kP1, kV1, kP2, kV2, kDeltaT, Qc6d),
                  std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
