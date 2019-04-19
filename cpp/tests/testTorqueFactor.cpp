/**
 * @file  testTorqueFactor.cpp
 * @brief test torque factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <TorqueFactor.h>
#include <DHLink.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example
{
// R link example
DH_Link dh_r = DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Vector3(0, 0, 0), -180, 10, 180);
// nosie model
noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Gaussian::Covariance(I_6x6);
Key torque_key = Symbol('t', 1),
    wrench_key = Symbol('F', 1);
} // namespace example

/**
 * Test Torque factor for stationary case
 */
TEST(TorqueFactor, error)
{
    // Create all factors
    Vector6 screw_axis;
    screw_axis << 0, 0, 1, 0, 1, 0;

    TorqueFactor factor(example::wrench_key, example::torque_key, example::cost_model, screw_axis);
    double torque = 20;
    Vector6 wrench;
    wrench << 0, 0, 10, 0, 10, 0;
    Vector1 actual_errors, expected_errors;
    Matrix actual_H1, actual_H2, expected_H1, expected_H2;

    actual_errors = factor.evaluateError(wrench, torque,actual_H1, actual_H2);
    expected_errors = Vector1(0);
    expected_H1 = numericalDerivative11(boost::function<Vector(const Vector6 &)>(
                                            boost::bind(&TorqueFactor::evaluateError, factor,
                                                        _1, torque, boost::none, boost::none)),
                                        wrench, 1e-6);
    expected_H2 = numericalDerivative11(boost::function<Vector(const double &)>(
                                            boost::bind(&TorqueFactor::evaluateError, factor,
                                                        wrench, _1, boost::none, boost::none)),
                                        torque, 1e-6);
    EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
    EXPECT(assert_equal(expected_H1, actual_H1, 1e-6));
    EXPECT(assert_equal(expected_H2, actual_H2, 1e-6));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
