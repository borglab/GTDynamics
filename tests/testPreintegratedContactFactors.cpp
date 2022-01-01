/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPreintegratedContactFactors.cpp
 * @brief Tests for the various preintegrated contact factors.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <math.h>

#include <iostream>

#include "gtdynamics/factors/PreintegratedContactFactors.h"

using namespace gtdynamics;
using namespace gtsam;
using gtdynamics::PoseKey;
using gtsam::assert_equal;

/* ************************************************************************* */
// Test constructor of Preintegrated Point Contact Measurements object.
// Used to perform forward integration for Preintegrated Point Contact Factor.
TEST(PreintegratedPointContactMeasurements, Constructor) {
  PreintegratedPointContactMeasurements();
  PreintegratedPointContactMeasurements pcm(
      Pose3(), Pose3(Rot3(), Vector3(0, 0, 1)), 0.01, I_3x3);
  EXPECT(assert_equal<Matrix3>(I_3x3 * 1e-4, pcm.preintMeasCov()));
}

/* ************************************************************************* */
// Test integration of a single measurement.
TEST(PreintegratedPointContactMeasurements, IntegrateMeasurement) {
  double dt = 0.01;
  PreintegratedPointContactMeasurements pcm(
      Pose3(), Pose3(Rot3(), Vector3(0, 0, 1)), dt, I_3x3);
  Rot3 deltaRik = Rot3::Ry(M_PI_4);
  Pose3 contact_k(Rot3(), Vector3(0, 0, 1));

  // Regression
  pcm.integrateMeasurement(deltaRik, contact_k, dt);
  EXPECT(assert_equal<Matrix3>(I_3x3 * 2e-4, pcm.preintMeasCov()));

  // Regression
  pcm.integrateMeasurement(deltaRik, Pose3(Rot3(), Vector3(0, 0, 1.01)), dt);
  EXPECT(assert_equal<Matrix3>(I_3x3 * 3e-4, pcm.preintMeasCov()));
}

/* ************************************************************************* */
// Test constructor for Preintegrated Point Contact Factor.
TEST(PreintegratedPointContactFactor, Constructor) {
  double dt = 0.01;
  PreintegratedPointContactMeasurements pcm(
      Pose3(), Pose3(Rot3(), Vector3(0, 0, 1)), dt, I_3x3);
  size_t base_id = 0, contact_id = 1;
  PreintegratedPointContactFactor ppcf(
      PoseKey(base_id, 0), PoseKey(contact_id, 0), PoseKey(base_id, 1),
      PoseKey(contact_id, 1), pcm);
}

/* ************************************************************************* */
// Test error function for Preintegrated Point Contact Factor.
TEST(PreintegratedPointContactFactor, Error) {
  size_t base_id = 0, contact_id = 1;
  size_t t0 = 0, t1 = 1;

  // Simple case where body moves forward 0.1 m/s and the foot is in fixed
  // contact but rotates by PI/4.
  // The rotation shouldn't have any effect.
  Pose3 wTb_i = Pose3(), wTc_i = Pose3(Rot3(), Point3(0, 0, 1)),
        wTb_j = Pose3(Rot3(), Point3(0.1, 0, 0)),
        wTc_j = Pose3(Rot3::Ry(M_PI_4), Point3(0, 0, 1));

  PreintegratedPointContactMeasurements pcm(wTb_i, wTc_i, 0.01, I_3x3);
  PreintegratedPointContactFactor factor(
      PoseKey(base_id, t0), PoseKey(contact_id, t0), PoseKey(base_id, t1),
      PoseKey(contact_id, t1), pcm);

  Vector3 error = factor.evaluateError(wTb_i, wTc_i, wTb_j, wTc_j);
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
}

/* ************************************************************************* */
// Test jacobians for Preintegrated Point Contact Factor.
TEST(PreintegratedPointContactFactor, Jacobians) {
  size_t base_id = 0, contact_id = 1;
  size_t t0 = 0, t1 = 1;

  // Simple case where body moves forward 0.1 m/s and the foot is in fixed
  // contact but rotates by PI/4.
  Pose3 wTb_i = Pose3(), wTc_i = Pose3(Rot3(), Point3(0, 0, 1)),
        wTb_j = Pose3(Rot3::Ry(M_PI_4), Point3(0.1, 0, 0)),
        wTc_j = Pose3(Rot3::Ry(M_PI_4), Point3(0, 0, 1));

  PreintegratedPointContactMeasurements pcm(wTb_i, wTc_i, 0.01, I_3x3);
  PreintegratedPointContactFactor factor(
      PoseKey(base_id, t0), PoseKey(contact_id, t0), PoseKey(base_id, t1),
      PoseKey(contact_id, t1), pcm);

  Values values;
  InsertPose(&values, base_id, t0, wTb_i);
  InsertPose(&values, base_id, t1, wTb_j);
  InsertPose(&values, contact_id, t0, wTc_i);
  InsertPose(&values, contact_id, t1, wTc_j);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Test constructor for Preintegrated Rigid Contact Factor.
TEST(PreintegratedRigidContactMeasurements, Constructor) {
  PreintegratedRigidContactMeasurements();

  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);
  // Covariance is 0 initially
  EXPECT(assert_equal<Matrix6>(Z_6x6, pcm.preintMeasCov()));
}

/* ************************************************************************* */
// Test if integrating a constant measurement for a Preintegrated Rigid Contact
// Measurements object works as expected.
TEST(PreintegratedRigidContactMeasurements, IntegrateMeasurementConstant) {
  double deltaT = 0.01;
  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);

  // Regression
  pcm.integrateMeasurement(deltaT);
  EXPECT(assert_equal<Matrix6>(I_6x6 * deltaT, pcm.preintMeasCov()));
}

/* ************************************************************************* */
// Test if integrating time varying measurements for a Preintegrated Rigid
// Contact Measurements object works as expected.
TEST(PreintegratedRigidContactMeasurements, IntegrateMeasurementVarying) {
  double dt = 0.01;
  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);

  Matrix6 expected;
  // Regression
  pcm.integrateMeasurement(I_3x3 * 0.05, I_3x3 * 0.01, dt);
  expected << I_3x3 * 0.05, Z_3x3, Z_3x3, I_3x3 * 0.01;
  EXPECT(assert_equal<Matrix6>(expected * dt * dt, pcm.preintMeasCov()));

  // Regression 2
  pcm.integrateMeasurement(I_3x3 * 0.06, I_3x3 * 0.02, dt);
  expected << I_3x3 * (0.05 + 0.06), Z_3x3, Z_3x3, I_3x3 * (0.01 + 0.02);
  EXPECT(assert_equal<Matrix6>(expected * dt * dt, pcm.preintMeasCov()));
}

/* ************************************************************************* */
// Test constructor for Preintegrated Rigid Contact Factor.
TEST(PreintegratedRigidContactFactor, Constructor) {
  size_t contact_id = 0;
  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);
  PreintegratedRigidContactFactor ppcf(PoseKey(contact_id, 0),
                                       PoseKey(contact_id, 1), pcm);
}

/* ************************************************************************* */
// Test error function for Preintegrated Rigid Contact Factor.
TEST(PreintegratedRigidContactFactor, Error) {
  size_t contact_id = 0;
  size_t t0 = 0, t1 = 1;

  // Simple case where the foot is in fixed contact but rotates forward by PI/4.
  Pose3 wTc_i = Pose3(Rot3(), Point3(0, 0, 1)),
        wTc_j = Pose3(Rot3::Ry(M_PI_4), Point3(0, 0, 1));

  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);
  // Integrate with constant noise
  pcm.integrateMeasurement(0.1);

  PreintegratedRigidContactFactor factor(PoseKey(contact_id, t0),
                                         PoseKey(contact_id, t1), pcm);

  Vector6 actual_error = factor.evaluateError(wTc_i, wTc_j);
  Vector6 expected_error;
  expected_error << 0, M_PI_4, 0, 0, 0, 0;
  EXPECT(assert_equal<Vector6>(expected_error, actual_error, 1e-9));
}

/* ************************************************************************* */
// Test jacobians for Preintegrated Rigid Contact Factor.
TEST(PreintegratedRigidContactFactor, Jacobians) {
  size_t contact_id = 0;
  size_t t0 = 0, t1 = 1;

  PreintegratedRigidContactMeasurements pcm(I_3x3, I_3x3);
  // Integrate with constant noise
  pcm.integrateMeasurement(1);

  PreintegratedRigidContactFactor factor(PoseKey(contact_id, t0),
                                         PoseKey(contact_id, t1), pcm);

  // Simple case.
  Pose3 wTc_i = Pose3(Rot3(), Point3(0, 0, 1)),
        wTc_j = Pose3(Rot3(), Point3(0, 0, 1));

  Values values;
  InsertPose(&values, contact_id, t0, wTc_i);
  InsertPose(&values, contact_id, t1, wTc_j);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
