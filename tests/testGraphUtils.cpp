/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testGraphUtils.cpp
 * @brief Test GraphUtils.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(GraphUtils, GraphDim) {
  NonlinearFactorGraph graph;
  graph.add(PriorFactor<double>(0, 0.0, noiseModel::Isotropic::Sigma(1, 1.0)));
  graph.add(PriorFactor<Vector2>(1, Vector2(0, 0), noiseModel::Isotropic::Sigma(2, 1.0)));

  EXPECT_LONGS_EQUAL(3, GraphDim(graph));
}

TEST(GraphUtils, IndexSet) {
  IndexSet indices;
  indices.insert(1);
  indices.insert(5);

  EXPECT(indices.exists(1));
  EXPECT(!indices.exists(2));
}

TEST(GraphUtils, IndexSetMap) {
  IndexSetMap index_map;
  Key key = 10;
  index_map.addIndex(key, 1);
  index_map.addIndex(key, 2);

  EXPECT(index_map.exists(key));
  EXPECT(index_map.at(key).exists(1));
  EXPECT(index_map.at(key).exists(2));
  EXPECT(!index_map.at(key).exists(3));

  IndexSetMap index_map2;
  index_map2.addIndex(key, 3);
  index_map.mergeWith(index_map2);
  EXPECT(index_map.at(key).exists(3));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
