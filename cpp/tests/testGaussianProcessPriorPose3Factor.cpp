/**
 *  @file testGaussianProcessPriorPose3Factor.cpp
 *  @test for Gaussian process prior linear factor for joint angle, joint
 *   velocity, and joint acceleration
 *  @author Mandy Xie
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <GaussianProcessPriorPose3Factor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

TEST(GaussianProcessPriorPose3Factor, Factor) {
  const double delta_t = 0.1;
  Matrix Qc = I_6x6;
  noiseModel::Gaussian::shared_ptr Qc_model =
      noiseModel::Gaussian::Covariance(Qc);
  Key p_key_1 = Symbol('p', 1), p_key_2 = Symbol('p', 2);
  Key v_key_1 = Symbol('v', 1), v_key_2 = Symbol('v', 2);
  Key a_key_1 = Symbol('a', 1), a_key_2 = Symbol('a', 2);
  GaussianProcessPriorPose3Factor factor(p_key_1, v_key_1, a_key_1, p_key_2,
                                         v_key_2, a_key_2, Qc_model, delta_t);
  Pose3 p1, p2;
  Vector6 v1, a1, v2, a2;
  v1.setZero(), a1.setZero(), v2.setZero(), a2.setZero();
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5, actualH6;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5, expectH6;
  Vector actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(p1, v1, a1, p2, v2, a2, actualH1, actualH2, actualH3,
                           actualH4, actualH5, actualH6);
  expected_errors = Vector(18).setZero();
  expectH1 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &GaussianProcessPriorPose3Factor::evaluateError,
                                factor, _1, v1, a1, p2, v2, a2)),
                            p1, 1e-6);
  expectH2 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&GaussianProcessPriorPose3Factor::evaluateError, factor,
                      p1, _1, a1, p2, v2, a2)),
      v1, 1e-6);
  expectH3 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&GaussianProcessPriorPose3Factor::evaluateError, factor,
                      p1, v1, _1, p2, v2, a2)),
      a1, 1e-6);
  expectH4 =
      numericalDerivative11(boost::function<Vector(const Pose3 &)>(boost::bind(
                                &GaussianProcessPriorPose3Factor::evaluateError,
                                factor, p1, v1, a1, _1, v2, a2)),
                            p2, 1e-6);
  expectH5 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&GaussianProcessPriorPose3Factor::evaluateError, factor,
                      p1, v1, a1, p2, _1, a2)),
      v2, 1e-6);
  expectH6 = numericalDerivative11(
      boost::function<Vector(const Vector6 &)>(
          boost::bind(&GaussianProcessPriorPose3Factor::evaluateError, factor,
                      p1, v1, a1, p2, v2, _1)),
      a2, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
