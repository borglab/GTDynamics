/**
 * @file  testToolPoseFactor.cpp
 * @brief test tool goal factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <ToolPoseFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example
{
// nosie model
noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Gaussian::Covariance(Matrix::Identity(6, 6));
Key pose_key = Symbol('p', 1);
} // namespace example

/**
 * Test tool pose factor
 */
TEST(ToolPoseFactor, error)
{
    // Create all factors
    Pose3 tTn = Pose3(Rot3(), Point3(-1, 0, 0));
    Pose3 tool_pose = Pose3(Rot3(), Point3(2, 0, 0));

    ToolPoseFactor factor(example::pose_key, example::cost_model, tTn, tool_pose);
    Pose3 pose(Rot3(), Point3(1, 0, 0));
    Vector actual_errors, expected_errors;
    Matrix actual_H, expected_H;

    actual_errors = factor.evaluateError(pose, actual_H);
    expected_errors = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
    expected_H = numericalDerivative11(boost::function<Vector(const Pose3 &)>(
                                            boost::bind(&ToolPoseFactor::evaluateError, factor,
                                                        _1, boost::none)),
                                        pose, 1e-6);
    EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
    EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
