/**
 *  @file testPneumaticActuator.cpp
 *  @test for pressure factor of pneumatic actuator
 *  @author Yetong
 **/
#include <PneumaticActuator.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace robot;

TEST(PneumaticActuator, Factor) {
  // coefficients for pressure factor
  const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
  const vector<double> pressureCoefficients {t0, c1, c2, c3};

  // coefficients for pneumatic actuator factor
  const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
               p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
               p12 = 0.0001081, p03 = -7.703e-07;
  const vector<double> pneumaticCoefficients {p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};

  // coefficients for actuator joint factor
  const double k = 1000;
  const double r = 0.1;
  const double qRest = 1;

  // setting
  const double init_p = 240;
  const double q = 1;
  const double current_time = 0.3;
  const int joint_idx = 0;

  PneumaticActuator actuator(joint_idx, pressureCoefficients, pneumaticCoefficients, k, r, qRest);
  double torque = actuator.calculateTorque(q, current_time, init_p);
  cout<<torque;
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
