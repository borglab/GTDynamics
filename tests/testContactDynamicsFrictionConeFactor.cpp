/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testContactDynamicsFrictionConeFactor.cpp
 * @brief test contact dynamics friction cone factor.
 * @Author: Alejandro Escontrela
 */

#include <math.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

#include "gtdynamics/factors/ContactDynamicsFrictionConeFactor.h"

using gtsam::assert_equal;

/**
 * Test the evaluateError method with various link contact wrenches and angles.
 **/
TEST(ContactDynamicsFrictionConeFactor, error) {
  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);

  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('p', 0, 0);
  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  double mu = 1.0;

  gtdynamics::ContactDynamicsFrictionConeFactor factor(
      pose_key, contact_wrench_key, cost_model, mu, gravity);

  // Link completely upright with contact wrench pointed normal to ground.
  EXPECT(
      assert_equal((gtsam::Vector(1) << 0).finished(),
                   factor.evaluateError(
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0., 0., 2.)),
                       (gtsam::Vector(6) << 0, 0, 0, 0, 0, 1).finished())));

  // Link completely upright with contact wrench pointed laterally to ground
  // (slip condition).
  EXPECT(
      assert_equal((gtsam::Vector(1) << 1).finished(),
                   factor.evaluateError(
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0., 0., 2.)),
                       (gtsam::Vector(6) << 0, 0, 0, 1, 0, 0).finished())));
  EXPECT(
      assert_equal((gtsam::Vector(1) << 11).finished(),
                   factor.evaluateError(
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0., 0., 2.)),
                       (gtsam::Vector(6) << 0, 0, 0, 4, 2, 3).finished())));

  // Rotation and translation.
  gtsam::Values values_simple;
  values_simple.insert(pose_key,
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 2.0)));
  values_simple.insert(contact_wrench_key,
                       (gtsam::Vector(6) << 0, 0, 0, 1, 0, 0).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(
      factor, values_simple,
      1e-7,   // Step used when computing numerical derivative jacobians.
      1e-3);  // Tolerance.

  // Link angled with contact wrench angled relative to ground.
  EXPECT(assert_equal(
      (gtsam::Vector(1) << 0).finished(),
      factor.evaluateError(
          gtsam::Pose3(gtsam::Rot3::Rx(M_PI / 8), gtsam::Point3(0., 0., 2.)),
          (gtsam::Vector(6) << 0, 0, 0, 0, 0, 1).finished())));
  EXPECT(assert_equal(
      (gtsam::Vector(1) << 0.7071).finished(),
      factor.evaluateError(gtsam::Pose3(gtsam::Rot3::Rx(3 * M_PI / 8),
                                        gtsam::Point3(0., 0., 2.)),
                           (gtsam::Vector(6) << 0, 0, 0, 0, 0, 1).finished()),
      1e-3));

  // Rotation and translation.
  gtsam::Values values_b;
  values_b.insert(pose_key, gtsam::Pose3(gtsam::Rot3::Rx(3 * M_PI / 8),
                                         gtsam::Point3(0., 0., 2.)));
  values_b.insert(contact_wrench_key,
                  (gtsam::Vector(6) << 0, 0, 0, 0, 0, 1).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(
      factor, values_b,
      1e-7,   // Step used when computing numerical derivative jacobians.
      1e-3);  // Tolerance.

  gtsam::Values values_c;
  values_c.insert(pose_key, gtsam::Pose3(gtsam::Rot3::RzRyRx(3 * M_PI / 8, 0.0,
                                                             3.5 * M_PI / 8),
                                         gtsam::Point3(0., 0., 2.)));
  values_c.insert(contact_wrench_key,
                  (gtsam::Vector(6) << 0, 0, 0, 0, 0, 1).finished());
  EXPECT_CORRECT_FACTOR_JACOBIANS(
      factor, values_c,
      1e-7,   // Step used when computing numerical derivative jacobians.
      1e-3);  // Tolerance.
}

/**
 * Test the optimization of a link contact wrench to ensure resulting wrench
 * lies within the friction cone.
 **/
TEST(ContactDynamicsFrictionConeFactor, optimization) {
  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);

  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('p', 0, 0);
  gtsam::LabeledSymbol contact_wrench_key = gtsam::LabeledSymbol('C', 0, 0);

  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  double mu = 1.0;

  gtdynamics::ContactDynamicsFrictionConeFactor factor(
      pose_key, contact_wrench_key, cost_model, mu, gravity);

  // Initial values.
  gtsam::Pose3 link_pose_init = gtsam::Pose3(
      gtsam::Pose3(gtsam::Rot3::Rx(3 * M_PI / 4), gtsam::Point3(0., 0., 2.)));
  gtsam::Vector6 contact_wrench_init =
      (gtsam::Vector(6) << 0, 0, 0, 2, 2, 1).finished();
  gtsam::Values init_values;
  init_values.insert(pose_key, link_pose_init);
  init_values.insert(contact_wrench_key, contact_wrench_init);

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the contact wrench and pose to obtain a contact wrench that lies
  // within the friction cone.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();

  gtsam::Values results = optimizer.values();
  gtsam::Pose3 link_pose_optimized = results.at(pose_key).cast<gtsam::Pose3>();
  gtsam::Vector6 contact_wrench_optimized =
      results.at(contact_wrench_key).cast<gtsam::Vector6>();

  std::cout << "Initial pose: " << std::endl << link_pose_init << std::endl;
  std::cout << "Initial wrench: " << std::endl
            << contact_wrench_init << std::endl;
  std::cout << "Initial error: " << std::endl
            << factor.evaluateError(link_pose_init, contact_wrench_init)
            << std::endl;

  std::cout << "Optimized pose: " << std::endl
            << link_pose_optimized << std::endl;
  std::cout << "Optimized wrench: " << std::endl
            << contact_wrench_optimized << std::endl;
  std::cout << "Optimized error: " << std::endl
            << factor.evaluateError(link_pose_optimized,
                                    contact_wrench_optimized)
            << std::endl;

  EXPECT(assert_equal(
      factor.evaluateError(link_pose_optimized, contact_wrench_optimized),
      (gtsam::Vector(1) << 0).finished(), 1e-3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
