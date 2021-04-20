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

auto kModel1 = Unit::Create(1);
auto kModel6 = Unit::Create(6);

TEST(ObjectiveFactors, PoseAndTwist) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  gtdynamics::add_link_objectives(&graph, id, k).pose(Pose3(), kModel6);
  EXPECT_LONGS_EQUAL(1, graph.size());
  gtdynamics::add_link_objectives(&graph, id, k)
      .pose(Pose3(), kModel6)
      .twist(gtsam::Z_6x1, kModel6);
  EXPECT_LONGS_EQUAL(3, graph.size());
}

TEST(ObjectiveFactors, TwistWithDerivatives) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  gtdynamics::add_link_objectives(&graph, id, k)
      .twist(gtsam::Z_6x1, kModel6)
      .twistAccel(gtsam::Z_6x1, kModel6);
  EXPECT_LONGS_EQUAL(2, graph.size());
}

TEST(ObjectiveFactors, JointAngleWithDerivatives) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  gtdynamics::add_joint_objectives(&graph, id, k).angle(0, kModel1);
  EXPECT_LONGS_EQUAL(1, graph.size());
  gtdynamics::add_joint_objectives(&graph, id, k)
      .velocity(0, kModel1)
      .acceleration(0, kModel1);
  EXPECT_LONGS_EQUAL(3, graph.size());
  gtdynamics::add_joint_objectives(&graph, id, k)
      .angle(0, kModel1)
      .velocity(0, kModel1)
      .acceleration(0, kModel1);
  EXPECT_LONGS_EQUAL(6, graph.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
