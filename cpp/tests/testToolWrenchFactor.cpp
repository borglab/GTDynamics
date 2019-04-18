/**
 * @file  testToolWrenchFactor.cpp
 * @brief test tool factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <DHLink.h>
#include <ToolWrenchFactor.h>

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
  external_wrench = (Vector(6) << 0, 0, 2, 0, 0, 0).finished();

  ToolWrenchFactor factor(example::twist_key, example::twist_accel_key,
                          example::wrench_j_key, example::pKey,
                          example::cost_model, tTn, inertia, external_wrench);
  Vector6 twist, twist_accel, wrench_j;
  twist = (Vector(6) << 0, 0, 10, 0, 1, 0).finished();
  twist_accel = (Vector(6) << 0, 0, 0, 0, 1, 0).finished();
  wrench_j = (Vector(6) << 0, 0, -2, -10, 1, 0).finished();
  Pose3 pose(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, expected_H1, expected_H2,
      expected_H3, expected_H4;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, pose, actual_H1,
                           actual_H2, actual_H3, actual_H4);
  expected_errors = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&ToolWrenchFactor::evaluateError, factor, _1, twist_accel,
                      wrench_j, pose)),
      twist, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(boost::bind(
          &ToolWrenchFactor::evaluateError, factor, twist, _1, wrench_j, pose)),
      twist_accel, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&ToolWrenchFactor::evaluateError, factor, twist,
                      twist_accel, _1, pose)),
      wrench_j, 1e-6);
  expected_H4 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &ToolWrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, _1)),
                            pose, 1e-6);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
}

/**
 * Test wrench factor with gravity
 */
TEST(ToolWrenchFactor, error_2) {
  // Create all factors
  Pose3 tTn = Pose3(Rot3(), Point3(-1, 0, 0));
  auto inertia = example::dh_r.inertiaMatrix();

  Vector6 external_wrench;
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();

  ToolWrenchFactor factor(example::twist_key, example::twist_accel_key,
                          example::wrench_j_key, example::pKey, example::cost_model, tTn,
                          inertia, external_wrench, gravity);

  Vector6 twist, twist_accel, wrench_j;
  twist = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  twist_accel = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  wrench_j = (Vector(6) << 0, 0, -2, 9.8, 0, 0).finished();
  Pose3 pose(Rot3::Rz(M_PI/2), Point3(0, 1, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, expected_H1, expected_H2,
      expected_H3, expected_H4;

  actual_errors =
      factor.evaluateError(twist, twist_accel, wrench_j, pose, actual_H1,
                           actual_H2, actual_H3, actual_H4);
  expected_errors = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&ToolWrenchFactor::evaluateError, factor, _1, twist_accel,
                      wrench_j, pose)),
      twist, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(boost::bind(
          &ToolWrenchFactor::evaluateError, factor, twist, _1, wrench_j, pose)),
      twist_accel, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&ToolWrenchFactor::evaluateError, factor, twist,
                      twist_accel, _1, pose)),
      wrench_j, 1e-6);
  expected_H4 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &ToolWrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, _1)),
                            pose, 1e-6);

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
