/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testObjectiveFactors.cpp
 * @brief Test creation of objectives.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::noiseModel::Unit;

TEST(ObjectiveFactors, PoseAndTwist) {
  constexpr int id = 5, k = 777;
  NonlinearFactorGraph graph;
  auto model = Unit::Create(6);
  gtdynamics::add_pose_objective(&graph, Pose3(), model, id, k);
  EXPECT_LONGS_EQUAL(1, graph.size());
  gtdynamics::add_link_objective(&graph, Pose3(), model, gtsam::Z_6x1, model,
                                 id, k);
  EXPECT_LONGS_EQUAL(3, graph.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
