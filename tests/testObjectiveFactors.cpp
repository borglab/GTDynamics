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
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <iostream>

using namespace gtdynamics;

using gtsam::assert_equal;
using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Vector3;
using gtsam::noiseModel::Unit;

auto kModel1 = Unit::Create(1);
auto kModel6 = Unit::Create(6);

TEST(ObjectiveFactors, PoseAndTwist) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  add_link_objectives(&graph, id, k).pose(Pose3(), kModel6);
  EXPECT_LONGS_EQUAL(1, graph.size());
  add_link_objectives(&graph, id, k)
      .pose(Pose3(), kModel6)
      .twist(gtsam::Z_6x1, kModel6);
  EXPECT_LONGS_EQUAL(3, graph.size());
}

TEST(ObjectiveFactors, TwistWithDerivatives) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  add_link_objectives(&graph, id, k)
      .twist(gtsam::Z_6x1, kModel6)
      .twistAccel(gtsam::Z_6x1, kModel6);
  EXPECT_LONGS_EQUAL(2, graph.size());
}

TEST(ObjectiveFactors, JointAngleWithDerivatives) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  add_joint_objectives(&graph, id, k).angle(0, kModel1);
  EXPECT_LONGS_EQUAL(1, graph.size());
  add_joint_objectives(&graph, id, k)
      .velocity(0, kModel1)
      .acceleration(0, kModel1);
  EXPECT_LONGS_EQUAL(3, graph.size());
  add_joint_objectives(&graph, id, k)
      .angle(0, kModel1)
      .velocity(0, kModel1)
      .acceleration(0, kModel1);
  EXPECT_LONGS_EQUAL(6, graph.size());
}

TEST(ObjectiveFactors, OptionalNoiseModels) {
  NonlinearFactorGraph graph;
  constexpr int id = 5, k = 777;
  add_joint_objectives(&graph, id, k).velocity(0).acceleration(0);
  add_joint_objectives(&graph, id, k).acceleration(0).angle(0).velocity(0);
  EXPECT_LONGS_EQUAL(5, graph.size());
}

TEST(Phase, AddGoals) {
  Robot robot =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"), "spider");

  // Foot is sphere of radius 1.1 cm, 14cm along X in COM
  Point3 point_com(0.14 + 0.011, 0, 0);

  // Predict goal point in world coordinates
  auto LF = robot.link("lower0");  // left forward leg
  auto bTcom = LF->wTcom();        // world is really body
  Point3 goal_point = bTcom.transformFrom(point_com);

  gtsam::NonlinearFactorGraph factors;
  unsigned char id = LF->id();
  size_t num_steps = 10;
  size_t k_start = 777;
  const gtsam::SharedNoiseModel &cost_model = nullptr;

  // Call AddStanceGoals function, creating 10 factors
  AddStanceGoals(&factors, num_steps, goal_point, cost_model, point_com, id,
                 k_start);
  EXPECT_LONGS_EQUAL(10, factors.size());

  auto f = boost::dynamic_pointer_cast<PointGoalFactor>(factors.back());

  // Regression, but realistic, at least in Z: 19.5 cm below body.
  constexpr double expected_height = 0.194667;
  Point3 expected(0.289324, 0.1575, -expected_height);
  EXPECT(assert_equal(expected, f->goalPoint(), 1e-5));

  // Check that prediction error is zero.
  EXPECT(assert_equal(Vector3(0, 0, 0), f->evaluateError(bTcom)));

  // Call AddSwingGoals function, creating 3 factors
  Point3 step(0.04, 0, 0);  // move by 4 centimeters in 3 steps
  gtsam::NonlinearFactorGraph swing_factors;
  AddSwingGoals(&swing_factors, goal_point, step, 3, cost_model, point_com, id);
  EXPECT_LONGS_EQUAL(3, swing_factors.size());

  // Last goal point should have moved just in front of goal_point
  auto g = boost::dynamic_pointer_cast<PointGoalFactor>(swing_factors.front());
  EXPECT(assert_equal<Point3>(
      expected + Point3(0.01, 0, expected_height - 0.159079), g->goalPoint(),
      1e-5));

  // Last goal point should have moved just shy of goal_point + step
  auto h = boost::dynamic_pointer_cast<PointGoalFactor>(swing_factors.back());
  EXPECT(assert_equal<Point3>(
      expected + step + Point3(-0.01, 0, expected_height - 0.139439),
      h->goalPoint(), 1e-5));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
