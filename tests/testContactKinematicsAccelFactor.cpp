/**
 * @file  testContactKinematicsAccelFactor.cpp
 * @brief test contact kinematics accel factor.
 * @Author: Alejandro Escontrela
 */

#include <ContactKinematicsAccelFactor.h>

#include <RobotModels.h>

#include <math.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using gtsam::assert_equal;

/**
 * Test the evaluateError method with various link twists.
 **/
TEST(ContactKinematicsAccelFactor, error) {

  using namespace simple_urdf;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);

  gtsam::LabeledSymbol twist_accel_key = gtsam::LabeledSymbol('A', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = my_robot.links()[0]->leTl_com();

  robot::ContactKinematicsAccelFactor factor(twist_accel_key, cost_model, cTcom);

  // A link with zero linear/angular accelration at its CoM should have zero
  // acceleration at the contact point.
  gtsam::Vector6 link_accel_stationary = (
      gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(
      factor.evaluateError(link_accel_stationary),
      (gtsam::Vector(3) << 0, 0, 0).finished()
  ));

  // A link with only a linear acceleration at its CoM should have a only a linear
  // accelration at the contact point (except in the case of a wheel for instance,
  // but this is a test intended for rigid links).
  gtsam::Vector6 link_accel_linear = (
      gtsam::Vector(6) << 0, 0, 0, 1, 1, 1).finished();
  EXPECT(assert_equal(
      factor.evaluateError(link_accel_linear),
      (gtsam::Vector(3) << 1, 1, 1).finished()
  ));

  // A link with only an angular acceleration component at the CoM should have a
  // linear accelration component at the contact point. Linear acceleration at contact
  // point a = r*alpha. With alpha = 1, and a distance from the Com to contact r = 1, the
  // linear component a = 1.
  gtsam::Vector6 link_accel_angular = (
      gtsam::Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(
      gtsam::norm_2(factor.evaluateError(link_accel_angular)),
      1.0
  ));

  // A link with both angular and linear acceleration at the CoM should have
  // a linear acceleration at the contact point (unless they cancel each other out).
  gtsam::Vector6 link_accel_angular_linear = (
      gtsam::Vector(6) << 2, 0, 0, 0, 0, 4).finished();
  EXPECT(assert_equal(
      gtsam::norm_2(factor.evaluateError(link_accel_angular_linear)),
      std::sqrt(std::pow(2, 2) + std::pow(4, 2))
  ));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Vector6 link_accel = (
      gtsam::Vector(6) << 1, 2, 4, 4, 9, 3).finished();
  values.insert(twist_accel_key, link_accel);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a link twist to ensure zero
 * velocity at the contact point.
 **/
TEST(ContactKinematicsAccelFactor, optimization) {

  using namespace simple_urdf;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(3);

  gtsam::LabeledSymbol twist_accel_key = gtsam::LabeledSymbol('V', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = my_robot.links()[0]->leTl_com();

  robot::ContactKinematicsAccelFactor factor(twist_accel_key, cost_model, cTcom);

  // Initial link twist.
  gtsam::Vector6 link_accel_init = (
      gtsam::Vector(6) << 2, 0, 0, 0, 0, 4).finished();
  
  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(twist_accel_key, link_accel_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();

  gtsam::Values results = optimizer.values();
  gtsam::Vector6 accel_optimized = results.at(twist_accel_key).cast<gtsam::Vector6>();

  EXPECT(assert_equal(
      factor.evaluateError(accel_optimized),
      (gtsam::Vector(3) << 0, 0, 0).finished()
  ));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
