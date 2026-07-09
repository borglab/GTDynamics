/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testGPInterpolators.cpp
 * @brief test Gaussian process interpolators, linear and Lie group versions.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/gpmp2/GPLieInterpolator.h>
#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/gpmp2/GPPose3Interpolator.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>

#include <functional>
#include <stdexcept>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Matrix;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector;
using gtsam::Vector2;
using gtsam::Vector6;
using gtsam::noiseModel::Isotropic;

static const double kDeltaT = 0.4;
static const double kTau = 0.15;

/* ************************* linear interpolator ************************* */

// Lambda and Psi must reproduce the support states exactly at the endpoints:
// Psi(0) = 0 and Lambda(0) = I, while Psi(dt) = I and Lambda(dt) = 0.
TEST(GPLinearInterpolator, reproducesEndpoints) {
  auto Qc_model = Isotropic::Sigma(2, 1.0);
  const Vector q1 = (Vector(2) << 1.0, -2.0).finished();
  const Vector v1 = (Vector(2) << 0.3, 0.7).finished();
  const Vector q2 = (Vector(2) << 2.5, 0.5).finished();
  const Vector v2 = (Vector(2) << -0.1, 0.2).finished();

  GPLinearInterpolator at_start(Qc_model, kDeltaT, 0.0);
  EXPECT(assert_equal(q1, at_start.interpolatePose(q1, v1, q2, v2), 1e-9));
  EXPECT(assert_equal(v1, at_start.interpolateVelocity(q1, v1, q2, v2), 1e-9));

  GPLinearInterpolator at_end(Qc_model, kDeltaT, kDeltaT);
  EXPECT(assert_equal(q2, at_end.interpolatePose(q1, v1, q2, v2), 1e-9));
  EXPECT(assert_equal(v2, at_end.interpolateVelocity(q1, v1, q2, v2), 1e-9));
}

// A minimum acceleration trajectory through a constant velocity pair is the
// straight line itself, so the interpolated state is exactly q1 + tau * v1.
TEST(GPLinearInterpolator, constantVelocityIsALine) {
  auto Qc_model = Isotropic::Sigma(2, 1.0);
  GPLinearInterpolator interp(Qc_model, kDeltaT, kTau);

  const Vector q1 = (Vector(2) << 1.0, -2.0).finished();
  const Vector v1 = (Vector(2) << 0.3, 0.7).finished();
  const Vector q2 = q1 + kDeltaT * v1;

  EXPECT(assert_equal(Vector(q1 + kTau * v1),
                      interp.interpolatePose(q1, v1, q2, v1), 1e-9));
  EXPECT(assert_equal(v1, interp.interpolateVelocity(q1, v1, q2, v1), 1e-9));
}

// numericalDerivative deduces its perturbation size from traits<X>::dimension,
// which is Eigen::Dynamic for gtsam::Vector, so the probed arguments are fixed
// size Vector2 here. They convert to the dynamic Vector the interpolator takes.
TEST(GPLinearInterpolator, jacobians) {
  auto Qc_model = Isotropic::Sigma(2, 1.0);
  GPLinearInterpolator interp(Qc_model, kDeltaT, kTau);

  const Vector2 q1(1.0, -2.0), v1(0.3, 0.7), q2(2.5, 0.5), v2(-0.1, 0.2);

  Matrix H1, H2, H3, H4;
  interp.interpolatePose(q1, v1, q2, v2, &H1, &H2, &H3, &H4);

  std::function<Vector2(const Vector2 &, const Vector2 &, const Vector2 &,
                        const Vector2 &)>
      pose = [&](const Vector2 &a, const Vector2 &b, const Vector2 &c,
                 const Vector2 &d) {
        return Vector2(interp.interpolatePose(a, b, c, d));
      };
  EXPECT(assert_equal(
      gtsam::numericalDerivative41<Vector2, Vector2, Vector2, Vector2, Vector2>(
          pose, q1, v1, q2, v2),
      H1, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative42<Vector2, Vector2, Vector2, Vector2, Vector2>(
          pose, q1, v1, q2, v2),
      H2, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative43<Vector2, Vector2, Vector2, Vector2, Vector2>(
          pose, q1, v1, q2, v2),
      H3, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative44<Vector2, Vector2, Vector2, Vector2, Vector2>(
          pose, q1, v1, q2, v2),
      H4, 1e-6));

  Matrix G1, G2, G3, G4;
  interp.interpolateVelocity(q1, v1, q2, v2, &G1, &G2, &G3, &G4);
  std::function<Vector2(const Vector2 &, const Vector2 &, const Vector2 &,
                        const Vector2 &)>
      vel = [&](const Vector2 &a, const Vector2 &b, const Vector2 &c,
                const Vector2 &d) {
        return Vector2(interp.interpolateVelocity(a, b, c, d));
      };
  EXPECT(assert_equal(
      gtsam::numericalDerivative41<Vector2, Vector2, Vector2, Vector2, Vector2>(
          vel, q1, v1, q2, v2),
      G1, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative42<Vector2, Vector2, Vector2, Vector2, Vector2>(
          vel, q1, v1, q2, v2),
      G2, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative43<Vector2, Vector2, Vector2, Vector2, Vector2>(
          vel, q1, v1, q2, v2),
      G3, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative44<Vector2, Vector2, Vector2, Vector2, Vector2>(
          vel, q1, v1, q2, v2),
      G4, 1e-6));
}

/* *************************** Lie interpolator *************************** */

TEST(GPLieInterpolator, rot3ReproducesEndpoints) {
  auto Qc_model = Isotropic::Sigma(3, 1.0);
  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Rot3 R2 = Rot3::RzRyRx(-0.3, 0.15, 0.05);
  const Vector v1 = (Vector(3) << 0.4, -0.1, 0.6).finished();
  const Vector v2 = (Vector(3) << 0.2, 0.3, -0.5).finished();

  GPLieInterpolator<Rot3> at_start(Qc_model, kDeltaT, 0.0);
  EXPECT(assert_equal(R1, at_start.interpolatePose(R1, v1, R2, v2), 1e-9));
  EXPECT(assert_equal(v1, at_start.interpolateVelocity(R1, v1, R2, v2), 1e-9));

  GPLieInterpolator<Rot3> at_end(Qc_model, kDeltaT, kDeltaT);
  EXPECT(assert_equal(R2, at_end.interpolatePose(R1, v1, R2, v2), 1e-9));
  EXPECT(assert_equal(v2, at_end.interpolateVelocity(R1, v1, R2, v2), 1e-9));
}

// Under a constant body twist the interpolated pose must be the exact geodesic
// point p1 * Expmap(tau * v1). This pins the lift, interpolate, retract path.
TEST(GPPose3Interpolator, constantTwistIsAGeodesic) {
  auto Qc_model = Isotropic::Sigma(6, 1.0);
  GPPose3Interpolator interp(Qc_model, kDeltaT, kTau);

  const Pose3 p1(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const Vector v1 = (Vector(6) << 0.1, -0.2, 0.3, 0.4, 0.5, -0.6).finished();
  const Pose3 p2 = p1 * Pose3::Expmap(kDeltaT * v1);

  EXPECT(assert_equal(Pose3(p1 * Pose3::Expmap(kTau * v1)),
                      interp.interpolatePose(p1, v1, p2, v1), 1e-9));
  EXPECT(assert_equal(v1, interp.interpolateVelocity(p1, v1, p2, v1), 1e-9));
}

TEST(GPPose3Interpolator, reproducesEndpoints) {
  auto Qc_model = Isotropic::Sigma(6, 1.0);
  const Pose3 p1(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const Pose3 p2(Rot3::RzRyRx(-0.2, 0.1, 0.4), Point3(1.4, 1.7, 3.2));
  const Vector v1 = (Vector(6) << 0.1, -0.2, 0.3, 0.4, 0.5, -0.6).finished();
  const Vector v2 = (Vector(6) << 0.0, 0.1, -0.1, 0.2, -0.3, 0.4).finished();

  GPPose3Interpolator at_start(Qc_model, kDeltaT, 0.0);
  EXPECT(assert_equal(p1, at_start.interpolatePose(p1, v1, p2, v2), 1e-9));
  EXPECT(assert_equal(v1, at_start.interpolateVelocity(p1, v1, p2, v2), 1e-9));

  GPPose3Interpolator at_end(Qc_model, kDeltaT, kDeltaT);
  EXPECT(assert_equal(p2, at_end.interpolatePose(p1, v1, p2, v2), 1e-9));
  EXPECT(assert_equal(v2, at_end.interpolateVelocity(p1, v1, p2, v2), 1e-9));
}

// As above, the velocity arguments are probed as fixed size Vector6.
TEST(GPPose3Interpolator, jacobians) {
  auto Qc_model = Isotropic::Sigma(6, 1.0);
  GPPose3Interpolator interp(Qc_model, kDeltaT, kTau);

  const Pose3 p1(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const Pose3 p2(Rot3::RzRyRx(-0.2, 0.1, 0.4), Point3(1.4, 1.7, 3.2));
  const Vector6 v1 = (Vector6() << 0.1, -0.2, 0.3, 0.4, 0.5, -0.6).finished();
  const Vector6 v2 = (Vector6() << 0.0, 0.1, -0.1, 0.2, -0.3, 0.4).finished();

  Matrix H1, H2, H3, H4;
  interp.interpolatePose(p1, v1, p2, v2, &H1, &H2, &H3, &H4);

  std::function<Pose3(const Pose3 &, const Vector6 &, const Pose3 &,
                      const Vector6 &)>
      pose = [&](const Pose3 &a, const Vector6 &b, const Pose3 &c,
                 const Vector6 &d) { return interp.interpolatePose(a, b, c, d); };
  EXPECT(assert_equal(
      gtsam::numericalDerivative41<Pose3, Pose3, Vector6, Pose3, Vector6>(
          pose, p1, v1, p2, v2),
      H1, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative42<Pose3, Pose3, Vector6, Pose3, Vector6>(
          pose, p1, v1, p2, v2),
      H2, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative43<Pose3, Pose3, Vector6, Pose3, Vector6>(
          pose, p1, v1, p2, v2),
      H3, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative44<Pose3, Pose3, Vector6, Pose3, Vector6>(
          pose, p1, v1, p2, v2),
      H4, 1e-6));

  Matrix G1, G2, G3, G4;
  interp.interpolateVelocity(p1, v1, p2, v2, &G1, &G2, &G3, &G4);
  std::function<Vector6(const Pose3 &, const Vector6 &, const Pose3 &,
                        const Vector6 &)>
      vel = [&](const Pose3 &a, const Vector6 &b, const Pose3 &c,
                const Vector6 &d) {
        return Vector6(interp.interpolateVelocity(a, b, c, d));
      };
  EXPECT(assert_equal(
      gtsam::numericalDerivative41<Vector6, Pose3, Vector6, Pose3, Vector6>(
          vel, p1, v1, p2, v2),
      G1, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative42<Vector6, Pose3, Vector6, Pose3, Vector6>(
          vel, p1, v1, p2, v2),
      G2, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative43<Vector6, Pose3, Vector6, Pose3, Vector6>(
          vel, p1, v1, p2, v2),
      G3, 1e-6));
  EXPECT(assert_equal(
      gtsam::numericalDerivative44<Vector6, Pose3, Vector6, Pose3, Vector6>(
          vel, p1, v1, p2, v2),
      G4, 1e-6));
}

// A Qc of the wrong dimension would mis-slice Lambda and Psi without crashing,
// so the constructor rejects it. Pose3 demands a 6 dimensional Qc.
TEST(GPPose3Interpolator, rejectsWrongQcDimension) {
  CHECK_EXCEPTION(GPPose3Interpolator(Isotropic::Sigma(3, 1.0), kDeltaT, kTau),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      GPLieInterpolator<Rot3>(Isotropic::Sigma(6, 1.0), kDeltaT, kTau),
      std::invalid_argument);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
