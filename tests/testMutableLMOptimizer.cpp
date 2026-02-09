/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTspaceBasis.cpp
 * @brief Test tagent space basis for constraint manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST(MutableLMOptimizer, optimize) {
  MutableLMOptimizer optimizer;

  Key x1_key = 1;
  Key x2_key = 2;
  NonlinearFactorGraph graph;
  auto noise = noiseModel::Unit::Create(6);
  graph.addPrior<Pose3>(x1_key, Pose3(Rot3(), Point3(0, 0, 0)), noise);
  graph.emplace_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);

  Values values;
  values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 0)));

  optimizer.setGraph(graph);
  optimizer.setValues(values);
  auto result = optimizer.optimize();

  Values expected_result;
  expected_result.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  expected_result.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  EXPECT(assert_equal(expected_result, result));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
