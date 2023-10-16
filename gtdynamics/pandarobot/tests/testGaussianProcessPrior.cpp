
/**
*  @file testGaussianProcessPrior.cpp
*  @author Wanli Qian, Toni Jubes, Frank Dellaert
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>


#include <gtdynamics/pandarobot/gpmp/GaussianProcessPrior.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;


/* ************************************************************************** */

TEST(GaussianProcessPrior, Optimization) {

  /**
   * A simple graph:
   *
   * p1   p2
   * |    |
   * x1   x2
   *  \  /
   *   gp
   *  /  \
   * v1  v2
   *
   * p1 and p2 are pose prior factor to fix the poses, gp is the GP factor
   * that get correct velocity of v1 and v2
   */

  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(3, 0.001);
  
  //noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  double theta1 = 0;
  double theta1d = 5;
  double theta2 = 4;
  double theta2d = 6;
  
  double EXPECTEDfi = 7;
  gtsam::Vector2 expected_error;
  expected_error << 1 , -1;

  double key1 = 0;
  double key2 = 1;
  double key3 = 2;
  double key4 = 3;
  double delta_t = 1;
  double Qc = 2;

  auto factor = GaussianProcessPrior(key1,key2,key3,key4,delta_t, Qc);
  gtsam::Values v;
  v.insert(key1,theta1);
  v.insert(key2,theta1d);
  v.insert(key3,theta2);
  v.insert(key4,theta2d);
  gtsam::Vector2 actual_error = factor.evaluateError(theta1,theta1d,theta2,theta2d);
  double ACTUALfi = factor.error(v);

EXPECT(assert_equal(expected_error,actual_error,1e-06))
EXPECT_DOUBLES_EQUAL(EXPECTEDfi,ACTUALfi,1e-06)

}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}