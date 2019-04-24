/**
 * @file  testWrenchFactor.cpp
 * @brief test wrench factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <DHLink.h>
#include <WrenchFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

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
    noiseModel::Gaussian::Covariance(Matrix::Identity(6, 6));
Key twist_key = Symbol('V', 1), twist_accel_key = Symbol('T', 1),
    wrench_j_key = Symbol('W', 1), wrench_k_key = Symbol('W', 2),
    qKey = Symbol('q', 1), pKey = Symbol('p', 1);
}  // namespace example

/**
 * Test wrench factor for stationary case with gravity
 */
TEST(WrenchFactor, error_1) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto inertia = example::dh_r.inertiaMatrix();
  Vector3 gravity;
  gravity << 0, -9.8, 0;

  WrenchFactor factor(example::twist_key, example::twist_accel_key,
                      example::wrench_j_key, example::wrench_k_key,
                      example::pKey, example::qKey, example::cost_model, kMj,
                      inertia, screw_axis, gravity);
  double q = M_PI / 4;
  Vector twist, twist_accel, wrench_j, wrench_k;
  twist = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twist_accel = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_j = (Vector(6) << 0, 0, 0, 0, 9.8, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  Pose3 pose = Pose3(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, wrench_k, pose, q);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::pKey, pose);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench factor for non-zero twist case, zero joint angle
 */
TEST(WrenchFactor, error_2) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto inertia = example::dh_r.inertiaMatrix();

  WrenchFactor factor(example::twist_key, example::twist_accel_key,
                      example::wrench_j_key, example::wrench_k_key,
                      example::pKey, example::qKey, example::cost_model, kMj,
                      inertia, screw_axis);
  double q = 0;
  Vector6 twist, twist_accel, wrench_j, wrench_k;
  twist = (Vector(6) << 0, 0, 1, 0, 1, 0).finished();
  twist_accel = (Vector(6) << 0, 0, 1, 0, 1, 0).finished();
  wrench_j = (Vector(6) << 0, 0, 4, -1, 2, 0).finished();
  wrench_k = (Vector(6) << 0, 0, 2, 0, 1, 0).finished();
  Pose3 pose = Pose3(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, wrench_k, pose, q);
  expected_errors << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::pKey, pose);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench factor for non-zero twist case, non-zero joint angle
 */
TEST(WrenchFactor, error_3) {
  // Create all factors
  Pose3 kMj = Pose3(Rot3(), Point3(-2, 0, 0));
  Vector6 screw_axis;
  screw_axis << 0, 0, 1, 0, 1, 0;
  auto inertia = example::dh_r.inertiaMatrix();

  Vector3 gravity;
  gravity << 0, -9.8, 0;

  WrenchFactor factor(example::twist_key, example::twist_accel_key,
                      example::wrench_j_key, example::wrench_k_key,
                      example::pKey, example::qKey, example::cost_model, kMj,
                      inertia, screw_axis, gravity);

  double q = M_PI / 4;
  Vector6 twist, twist_accel, wrench_j, wrench_k;
  twist = (Vector(6) << 0, 0, 10, 0, 10, 0).finished();
  twist_accel = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_j = (Vector(6) << 0, 0, 7.07106781, -107.07106781 + 9.8, 7.07106781, 0).finished();
  wrench_k = (Vector(6) << 0, 0, -10, 0, 10, 0).finished();
  Pose3 pose = Pose3(Rot3::Rz(M_PI / 2), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, wrench_k, pose, q);
  expected_errors << 0, 0, 0, 0, 0, 0;

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::wrench_k_key, wrench_k);
  values.insert(example::pKey, pose);
  values.insert(example::qKey, q);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
