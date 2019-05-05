/**
 *  @file  testPaddleFactor.cpp
 *  @brief Unit tests for PaddleFactor Class
 *  @author Stephen Eick
 *  @author Frank Dellaert
 *  @date April 2019
 */

#include <gtsam_unstable/nonlinear/PaddleFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/expressionTesting.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(PaddleFactor, Creation)
{
  const Key key = 0;
  const double t = 0.25;
  const double omega = 6.283;
  const double length = 1;
  const double alpha = 1;
  const double beta = 1;
  PaddleFactor factor(key, t, omega, length, alpha, beta);

  // Write a test to verify the error

  // Set the linearization point
  Values values;
  //values.insert(key, Vector3(1.0, 2.0, 0.57));
  //values.insert(key, Vector3(1.5661, 1.2717, 1.2717));
  values.insert(key, Vector3(1.5662, 1.2717, 1.2717));

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
