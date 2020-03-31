/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testContactKinematicsTwistFactor.cpp
 * @brief test contact kinematics twist factor.
 * @Author: Alejandro Escontrela
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

#include "gtdynamics/factors/ContactKinematicsTwistFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using gtsam::assert_equal;

/**
 * Test the evaluateError method with various link twists.
 **/
TEST(ContactKinematicsTwistFactor, error) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);

  gtsam::LabeledSymbol twist_key = gtsam::LabeledSymbol('V', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1));
  gtdynamics::ContactKinematicsTwistFactor factor(twist_key, cost_model, cTcom);

  // A link with zero linear/angular velocity at its CoM should have a
  // stationary contact point.
  gtsam::Vector6 link_twist_stationary =
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(factor.evaluateError(link_twist_stationary),
                      (gtsam::Vector(3) << 0, 0, 0).finished()));

  // A link with only a linear velocity at its CoM should have a only a linear
  // velocity at the contact point (except in the case of a wheel for instance,
  // but this is a test intended for rigid links).
  gtsam::Vector6 link_twist_linear =
      (gtsam::Vector(6) << 0, 0, 0, 1, 1, 1).finished();
  EXPECT(assert_equal(factor.evaluateError(link_twist_linear),
                      (gtsam::Vector(3) << 1, 1, 1).finished()));

  // A link with only an angular velocity component at the CoM should have a
  // linear velocity component at the contact point. Linear velocity at contact
  // point v = rw. With w = 1, and a distance from the Com to contact r = 1, the
  // linear component v = 1.
  gtsam::Vector6 link_twist_angular =
      (gtsam::Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  EXPECT(assert_equal(factor.evaluateError(link_twist_angular).norm(),
                      1.0));

  // A link with both angular velocity and linear velocity at the CoM should
  // have a linear velocity at the contact point (unless they cancel each other
  // out).
  gtsam::Vector6 link_twist_angular_linear =
      (gtsam::Vector(6) << 2, 0, 0, 0, 0, 4).finished();
  EXPECT(assert_equal(
      factor.evaluateError(link_twist_angular_linear).norm(),
      std::sqrt(std::pow(2, 2) + std::pow(4, 2))));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Vector6 link_twist = (gtsam::Vector(6) << 1, 2, 4, 4, 9, 3).finished();
  values.insert(twist_key, link_twist);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a link twist to ensure zero
 * velocity at the contact point.
 **/
TEST(ContactKinematicsTwistFactor, optimization) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(3);

  gtsam::LabeledSymbol twist_key = gtsam::LabeledSymbol('V', 0, 0);

  // Transform from the robot com to the contact point.
  gtsam::Pose3 cTcom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1));
  gtdynamics::ContactKinematicsTwistFactor factor(twist_key, cost_model, cTcom);

  // Initial link twist.
  gtsam::Vector6 link_twist_init =
      (gtsam::Vector(6) << 2, 0, 0, 0, 0, 4).finished();

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(twist_key, link_twist_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();

  gtsam::Values results = optimizer.values();
  gtsam::Vector6 twist_optimized = results.at(twist_key).cast<gtsam::Vector6>();

  EXPECT(assert_equal(factor.evaluateError(twist_optimized),
                      (gtsam::Vector(3) << 0, 0, 0).finished()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
