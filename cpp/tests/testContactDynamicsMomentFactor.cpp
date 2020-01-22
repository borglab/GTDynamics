/**
 * @file  testContactDynamicsMomentFactor.cpp
 * @brief test contact dynamics moment factor.
 * @Author: Alejandro Escontrela
 */

#include <ContactDynamicsMomentFactor.h>

#include <RobotModels.h>

#include <math.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;

/**
 * Test the evaluateError method with various contact wrenches.
 **/
TEST(ContactDynamicsMomentFactor, error) {

  using namespace simple_urdf;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(I_3x3);

  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = my_robot.links()[0]->leTl_com();

  robot::ContactDynamicsMomentFactor factor(
      contact_wrench_key, cost_model, cTcom);

  // A link with zero contact wrench should have zero
  // moment at the contact point.
  gtsam::Vector6 zero_wrench = (
      gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(
      factor.evaluateError(zero_wrench),
      (gtsam::Vector(3) << 0, 0, 0).finished()
  ));

  // A link with 5N contact force in the x direction (in the contact
  // frame) and a -5Nm moment in the y direction (in the COM frame)
  // expressed no linear force in the com frame but the moments remain.
  gtsam::Vector6 link_wrench_linear = (
      gtsam::Vector(6) << 0, -5, 0, 5, 0, 0).finished();
  EXPECT(assert_equal(
      factor.evaluateError(link_wrench_linear),
      (gtsam::Vector(3) << 0, -5, 0).finished()
  ));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Vector6 link_wrench = (
      gtsam::Vector(6) << 1, 2, 4, 4, 9, 3).finished();
  values.insert(contact_wrench_key, link_wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a contact wrench to ensure zero
 * moment at the contact point.
 **/
TEST(ContactDynamicsMomentFactor, optimization) {

  using namespace simple_urdf;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(3);

  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = my_robot.links()[0]->leTl_com();

  robot::ContactKinematicsAccelFactor factor(
      contact_wrench_key, cost_model, cTcom);

  // Initial link twist.
  gtsam::Vector6 contact_wrench_init = (
      gtsam::Vector(6) << 2, 8, -5, 7, 12, 4).finished();
  
  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(contact_wrench_key, contact_wrench_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();

  gtsam::Values results = optimizer.values();
  gtsam::Vector6 contact_wrench_optimized = results.at(
      contact_wrench_key).cast<gtsam::Vector6>();

  std::cout << "Initial Contact Wrench: " << std::endl;
  std::cout << contact_wrench_init << std::endl;

  std::cout << "Initial Contact Wrench Error: " << std::endl;
  std::cout << factor.evaluateError(contact_wrench_init) << std::endl;

  std::cout << "Optimized Contact Wrench: " << std::endl; 
  std::cout << contact_wrench_optimized << std::endl;

  std::cout << "Optimized Contact Wrench Error: " << std::endl;
  std::cout << factor.evaluateError(contact_wrench_optimized) << std::endl;

  EXPECT(assert_equal(
      factor.evaluateError(contact_wrench_optimized),
      (gtsam::Vector(3) << 0, 0, 0).finished()
  ));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
