/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testContactDynamicsMomentFactor.cpp
 * @brief test contact dynamics moment factor.
 * @author Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <math.h>

#include <iostream>

#include "gtdynamics/factors/ContactDynamicsMomentFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using gtsam::assert_equal;

/**
 * Test the evaluateError method with various contact wrenches.
 **/
TEST(ContactDynamicsMomentFactor, error) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);

  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1));
  ContactDynamicsMomentFactor factor(contact_wrench_key, cost_model, cTcom);

  // A link with zero contact wrench should have zero
  // moment at the contact point.
  gtsam::Vector6 zero_wrench =
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(factor.evaluateError(zero_wrench),
                      (gtsam::Vector(3) << 0, 0, 0).finished()));

  // A link with 5N contact force in the x direction (in the contact
  // frame) and a -5Nm moment in the y direction (in the COM frame)
  // expressed no linear force in the com frame but the moments remain.
  gtsam::Vector6 link_wrench_linear =
      (gtsam::Vector(6) << 0, -5, 0, 5, 0, 0).finished();
  EXPECT(assert_equal(factor.evaluateError(link_wrench_linear),
                      (gtsam::Vector(3) << 0, -10, 0).finished()));

  // Now the moment should some to zero.
  gtsam::Vector6 link_wrench_linear_2 =
      (gtsam::Vector(6) << 0, -5, 0, -5, 0, 0).finished();
  EXPECT(assert_equal(factor.evaluateError(link_wrench_linear_2),
                      (gtsam::Vector(3) << 0, 0, 0).finished()));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Vector6 link_wrench =
      (gtsam::Vector(6) << 1, 2, 4, 4, 9, 3).finished();
  values.insert(contact_wrench_key, link_wrench);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a contact wrench to ensure zero
 * moment at the contact point.
 **/
TEST(ContactDynamicsMomentFactor, optimization) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(3);

  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1));
  ContactDynamicsMomentFactor factor(contact_wrench_key, cost_model, cTcom);

  // Initial link twist.
  gtsam::Vector6 contact_wrench_init =
      (gtsam::Vector(6) << 2, 8, -5, 7, 12, 4).finished();

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
  gtsam::Vector6 contact_wrench_optimized =
      results.at<gtsam::Vector6>(contact_wrench_key);

  std::cout << "Initial Contact Wrench: " << std::endl;
  std::cout << contact_wrench_init << std::endl;

  std::cout << "Initial Contact Wrench Error: " << std::endl;
  std::cout << factor.evaluateError(contact_wrench_init) << std::endl;

  std::cout << "Optimized Contact Wrench: " << std::endl;
  std::cout << contact_wrench_optimized << std::endl;

  std::cout << "Optimized Contact Wrench Error: " << std::endl;
  std::cout << factor.evaluateError(contact_wrench_optimized) << std::endl;

  EXPECT(assert_equal(factor.evaluateError(contact_wrench_optimized),
                      (gtsam::Vector(3) << 0, 0, 0).finished()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
