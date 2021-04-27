/**
 *  @file testPneumaticActuator.cpp
 *  @test for pressure factor of pneumatic actuator
 *  @author Yetong
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

#include "gtdynamics/jumpingrobot/factors/PneumaticActuator.h"

using namespace std;
using namespace gtsam;
using namespace gtdynamics;

TEST(PneumaticActuator, computeResult) {

  // PneumaticActuator::Params params;
  // params.joint_name = "j1";
  // params.j = 1;                          // joint index
  // params.kt = 8200;                      // spring coefficient for tendon spring
  // params.ka = 2.0;                      // spring coefficient for antagonistic spring
  // params.q_rest = 0;                  // nominal angle without contraction
  // params.q_anta_limit =0;            // joint limit to activate antagonistic spring
  // params.r = 0.04;                       // radius of pulley r
  // params.b = 0.05;                       // damping coefficient
  // params.positive = false;                  // actuator configuration

  // params.Rs = 287.0550;
  // params.T = 296.15;
  // params.D = 0.1575 * 0.0254;
  // params.L = 74 * 0.0254;
  // params.mu = 1.8377e-5;
  // params.epsilon = 1e-5;
  // params.ct = 1e-3;

  PneumaticActuator actuator = PneumaticActuator();

  PneumaticActuator::PriorValues prior_values = actuator.priorValues();
  // prior_values.q = 0.0;
  // prior_values.m = 7.873172488131229e-05;
  // prior_values.v = 0.0;
  // prior_values.Ps = 65.0 * 6.89476;
  // prior_values.t = 0.0;
  // prior_values.to = 0.0;
  // prior_values.tc = 1.0;
  int k = 0;
  // double vs = 1.475e-3;
  // double ms = prior_values.Ps * 1000 * vs / (actuator.params().Rs * actuator.params().T);
 


  gtsam::NonlinearFactorGraph graph;
  graph.push_back(actuator.actuatorFactorGraph(k));
  // graph.push_back(actuator.sourceFactorGraph(k));
  graph.push_back(actuator.actuatorPriorGraph(k, prior_values));
  // graph.push_back(actuator.sourcePriorGraph(k, ms, vs));

  gtsam::Values init_values;
  init_values.insert(actuator.actuatorInitValues(k, prior_values));
  // init_values.insert(actuator.sourceInitValues(k, ms, vs));

  // graph.print("", GTDKeyFormatter);
  // init_values.print("", GTDKeyFormatter);

  // for (auto factor: graph) {
  //   factor->print("", GTDKeyFormatter);
  //   std::cout << "error: " << factor->error(init_values) << std::endl;
  // }

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setLinearSolverType("MULTIFRONTAL_QR");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, lm_params);
  gtsam::Values result = optimizer.optimize();
  // gtsam::Values previous_values;
  // auto result = actuator.computeResult(k, prior_values, previous_values);

  // result.print();
  // EXPECT(assert_equal(-5.07465, torque, 1e-3));
}


// TEST(PneumaticActuator, computeResult) {
//   // coefficients for pressure factor
//   const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
//   const vector<double> pressureCoefficients {t0, c1, c2, c3};

//   // coefficients for pneumatic actuator factor
//   const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
//                p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
//                p12 = 0.0001081, p03 = -7.703e-07;
//   const vector<double> pneumaticCoefficients {p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};

//   // coefficients for actuator joint factor
//   const double k = 5000;
//   const double r = 0.02;
//   const double qRest = M_PI_2;

//   // setting
//   const double init_p = 241.3;
//   const double q = 1.59225;
//   const double start_time = 0;
//   const double current_time = 0.0395;
//   const int j = 1;
//   const std::string joint_nanme = "j1";
//   bool contract = false;
//   int t = 0;

//   PneumaticActuator actuator(joint_nanme, j, pressureCoefficients, pneumaticCoefficients, k, r, qRest, contract);
//   auto result = actuator.computeResult(t, q, start_time, current_time, init_p);
//   // EXPECT(assert_equal(-5.07465, torque, 1e-3));
// }

// TEST(PneumaticActuator, Factor) {
//   // coefficients for pressure factor
//   const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
//   const vector<double> pressureCoefficients {t0, c1, c2, c3};

//   // coefficients for pneumatic actuator factor
//   const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
//                p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
//                p12 = 0.0001081, p03 = -7.703e-07;
//   const vector<double> pneumaticCoefficients {p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};

//   // coefficients for actuator joint factor
//   const double k = 1000;
//   const double r = 0.1;
//   const double qRest = 1;

//   // setting
//   const double init_p = 240;
//   const double q = 1;
//   const double current_time = 0.3;
//   const int j = 1;
//   const std::string joint_nanme = "j1";

//   PneumaticActuator actuator(joint_nanme, j, pressureCoefficients, pneumaticCoefficients, k, r, qRest);
//   double torque = actuator.calculateTorque(q, current_time, init_p);
//   EXPECT(assert_equal(-5.07465, torque, 1e-3));
// }

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
