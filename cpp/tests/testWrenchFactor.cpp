/**
 * @file  testWrenchFactor.cpp
 * @brief test wrench factor
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
#include <WrenchFactor.h>

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
  Vector6 twist, twist_accel, wrench_j, wrench_k;
  twist << 0, 0, 0, 0, 0, 0;
  twist_accel << 0, 0, 0, 0, 0, 0;
  wrench_j << 0, 0, 0, 0, 9.8, 0;
  wrench_k << 0, 0, 0, 0, 0, 0;
  Pose3 pose = Pose3(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, actual_H5, actual_H6,
      expected_H1, expected_H2, expected_H3, expected_H4, expected_H5,
      expected_H6;

  actual_errors = factor.evaluateError(twist, twist_accel, wrench_j, wrench_k,
                                       pose, q, actual_H1, actual_H2, actual_H3,
                                       actual_H4, actual_H5, actual_H6);
  expected_errors << 0, 0, 0, 0, 0, 0;
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, _1, twist_accel,
                      wrench_j, wrench_k, pose, q)),
      twist, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, _1, wrench_j,
                      wrench_k, pose, q)),
      twist_accel, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      _1, wrench_k, pose, q)),
      wrench_j, 1e-6);
  expected_H4 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      wrench_j, _1, pose, q)),
      wrench_k, 1e-6);
  expected_H5 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, _1, q)),
                            pose, 1e-6);
  expected_H6 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, pose, _1)),
                            q, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
  EXPECT(assert_equal(expected_H5, actual_H5, 1e-6));
  EXPECT(assert_equal(expected_H6, actual_H6, 1e-6));
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
  twist << 0, 0, 1, 0, 1, 0;
  twist_accel << 0, 0, 1, 0, 1, 0;
  wrench_j << 0, 0, 4, -1, 2, 0;
  wrench_k << 0, 0, 2, 0, 1, 0;
  Pose3 pose = Pose3(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, actual_H5, actual_H6,
      expected_H1, expected_H2, expected_H3, expected_H4, expected_H5,
      expected_H6;

  actual_errors = factor.evaluateError(twist, twist_accel, wrench_j, wrench_k,
                                       pose, q, actual_H1, actual_H2, actual_H3,
                                       actual_H4, actual_H5, actual_H6);
  expected_errors << 0, 0, 0, 0, 0, 0;
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, _1, twist_accel,
                      wrench_j, wrench_k, pose, q)),
      twist, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, _1, wrench_j,
                      wrench_k, pose, q)),
      twist_accel, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      _1, wrench_k, pose, q)),
      wrench_j, 1e-6);
  expected_H4 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      wrench_j, _1, pose, q)),
      wrench_k, 1e-6);
  expected_H5 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, _1, q)),
                            pose, 1e-6);
  expected_H6 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, pose, _1)),
                            q, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
  EXPECT(assert_equal(expected_H5, actual_H5, 1e-6));
  EXPECT(assert_equal(expected_H6, actual_H6, 1e-6));
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
  twist << 0, 0, 10, 0, 10, 0;
  twist_accel << 0, 0, 0, 0, 0, 0;
  wrench_j << 0, 0, 7.07106781, -107.07106781 + 9.8, 7.07106781, 0;
  wrench_k << 0, 0, -10, 0, 10, 0;
  Pose3 pose = Pose3(Rot3::Rz(M_PI / 2), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H1, actual_H2, actual_H3, actual_H4, actual_H5, actual_H6,
      expected_H1, expected_H2, expected_H3, expected_H4, expected_H5,
      expected_H6;

  actual_errors = factor.evaluateError(twist, twist_accel, wrench_j, wrench_k,
                                       pose, q, actual_H1, actual_H2, actual_H3,
                                       actual_H4, actual_H5, actual_H6);
  expected_errors << 0, 0, 0, 0, 0, 0;
  expected_H1 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, _1, twist_accel,
                      wrench_j, wrench_k, pose, q)),
      twist, 1e-6);
  expected_H2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, _1, wrench_j,
                      wrench_k, pose, q)),
      twist_accel, 1e-6);
  expected_H3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      _1, wrench_k, pose, q)),
      wrench_j, 1e-6);
  expected_H4 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&WrenchFactor::evaluateError, factor, twist, twist_accel,
                      wrench_j, _1, pose, q)),
      wrench_k, 1e-6);
  expected_H5 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, _1, q)),
                            pose, 1e-6);
  expected_H6 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &WrenchFactor::evaluateError, factor, twist,
                                twist_accel, wrench_j, wrench_k, pose, _1)),
                            q, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
  EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
  EXPECT(assert_equal(expected_H3, actual_H3, 1e-6));
  EXPECT(assert_equal(expected_H4, actual_H4, 1e-6));
  EXPECT(assert_equal(expected_H5, actual_H5, 1e-6));
  EXPECT(assert_equal(expected_H6, actual_H6, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
