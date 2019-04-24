/**
 * @file  testToolWrenchFactor.cpp
 * @brief test tool factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <DHLink.h>
#include <ToolWrenchFactor.h>

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
    noiseModel::Gaussian::Covariance(I_6x6);
Key twist_key = Symbol('V', 1), twist_accel_key = Symbol('T', 1),
    wrench_j_key = Symbol('W', 1), pKey = Symbol('p', 1);
}  // namespace example

/**
 * Test wrench factor for gravity compensation and non-zero twist case
 */
TEST(ToolWrenchFactor, error_1) {
  // Create all factors
  Pose3 tTn = Pose3(Rot3(), Point3(-1, 0, 0));
  auto inertia = example::dh_r.inertiaMatrix();
  Vector6 external_wrench;
  external_wrench << 0, 0, 2, 0, 0, 0;

  ToolWrenchFactor factor(example::twist_key, example::twist_accel_key,
                          example::wrench_j_key, example::pKey,
                          example::cost_model, tTn, inertia, external_wrench);
  Vector6 twist, twist_accel, wrench_j;
  twist << 0, 0, 10, 0, 1, 0;
  twist_accel << 0, 0, 0, 0, 1, 0;
  wrench_j << 0, 0, -2, -10, 1, 0;
  Pose3 pose(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test wrench factor with gravity
 */
TEST(ToolWrenchFactor, error_2) {
  // Create all factors
  Pose3 tTn = Pose3(Rot3(), Point3(-1, 0, 0));
  auto inertia = example::dh_r.inertiaMatrix();

  Vector6 external_wrench;
  Vector3 gravity;
  gravity << 0, -9.8, 0;

  ToolWrenchFactor factor(example::twist_key, example::twist_accel_key,
                          example::wrench_j_key, example::pKey, example::cost_model, tTn,
                          inertia, external_wrench, gravity);

  Vector6 twist, twist_accel, wrench_j;
  twist << 0, 0, 0, 0, 0, 0;
  twist_accel << 0, 0, 0, 0, 0, 0;
  wrench_j << 0, 0, -2, 9.8, 0, 0;
  Pose3 pose(Rot3::Rz(M_PI/2), Point3(0, 1, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::twist_key, twist);
  values.insert(example::twist_accel_key, twist_accel);
  values.insert(example::wrench_j_key, wrench_j);
  values.insert(example::pKey, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
