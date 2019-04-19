/**
 *  @file testGaussianProcessPriorFactor.cpp
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

#include <GaussianProcessPriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

TEST(GaussianProcessPriorFactor, Factor) {
  const double delta_t = 0.1;
  Matrix Qc = I_1x1;
  noiseModel::Gaussian::shared_ptr Qc_model =
      noiseModel::Gaussian::Covariance(Qc);
  Key q_key_1 = Symbol('x', 1), q_key_2 = Symbol('x', 2);
  Key qVel_key_1 = Symbol('v', 1), qVel_key_2 = Symbol('v', 2);
  Key qAccel_key_1 = Symbol('a', 1), qAccel_key_2 = Symbol('a', 2);
  GaussianProcessPriorFactor factor(q_key_1, qVel_key_1, qAccel_key_1, q_key_2,
                                    qVel_key_2, qAccel_key_2, Qc_model,
                                    delta_t);
  double q1 = 0, qVel1 = 0, qAccel1 = 0, q2 = 0, qVel2 = 0, qAccel2 = 0;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5, actualH6;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5, expectH6;
  Vector3 actual_errors, expected_errors;

  actual_errors =
      factor.evaluateError(q1, qVel1, qAccel1, q2, qVel2, qAccel2, actualH1,
                           actualH2, actualH3, actualH4, actualH5, actualH6);
  expected_errors.setZero();
  expectH1 = numericalDerivative11(
      boost::function<Vector(const double &)>(
          boost::bind(&GaussianProcessPriorFactor::evaluateError, factor, _1,
                      qVel1, qAccel1, q2, qVel2, qAccel2)),
      q1, 1e-6);
  expectH2 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &GaussianProcessPriorFactor::evaluateError,
                                factor, q1, _1, qAccel1, q2, qVel2, qAccel2)),
                            qVel1, 1e-6);
  expectH3 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &GaussianProcessPriorFactor::evaluateError,
                                factor, q1, qVel1, _1, q2, qVel2, qAccel2)),
                            qAccel1, 1e-6);
  expectH4 = numericalDerivative11(
      boost::function<Vector(const double &)>(
          boost::bind(&GaussianProcessPriorFactor::evaluateError, factor, q1,
                      qVel1, qAccel1, _1, qVel2, qAccel2)),
      q2, 1e-6);
  expectH5 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &GaussianProcessPriorFactor::evaluateError,
                                factor, q1, qVel1, qAccel1, q2, _1, qAccel2)),
                            qVel2, 1e-6);
  expectH6 =
      numericalDerivative11(boost::function<Vector(const double &)>(boost::bind(
                                &GaussianProcessPriorFactor::evaluateError,
                                factor, q1, qVel1, qAccel1, q2, qVel2, _1)),
                            qAccel2, 1e-6);
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
