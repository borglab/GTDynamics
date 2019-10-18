/**
 * @file  testIK.cpp
 * @brief test inverse kinematics factor graph
 * @Author: Mandy Xie
 */

#include <Arm.h>
#include <BasePoseFactor.h>
#include <DHLink.h>
#include <MotionPlanner.h>
#include <PoseFactor.h>
#include <ToolPoseFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// Test inverse kinematics
TEST(IK_factor_graph, optimization) {
  // RR link example
  vector<DH_Link> dh_rr = {DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Z_3x3),
                           DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0),
                                   Z_3x3)};
  auto robot = Arm<DH_Link>(dh_rr);
  Pose3 pose_goal(Pose3(Rot3::Rz(M_PI / 2), Point3(0, 4, 0)));
  auto dof = robot.numLinks();

  // get robot jTi list at rest
  auto jMi = robot.jTis(Vector::Zero(dof));
  // get base pose in world frame
  auto base_pose = robot.base();
  // get link COM pose at start
  auto poses = robot.comFrames();
  // get robot screw_axes for all links
  auto screw_axes = robot.screwAxes();

  gtsam::noiseModel::Base::shared_ptr bp_cost_model =
                                          noiseModel::Constrained::All(6),
                                      p_cost_model =
                                          noiseModel::Constrained::All(6),
                                      tp_cost_model =
                                          noiseModel::Constrained::All(6);

  NonlinearFactorGraph graph;

  // add base pose factor
  graph.add(BasePoseFactor(PoseKey(0, 0), bp_cost_model, base_pose));

  for (int j = 1; j <= dof; ++j) {
    // add pose factor
    graph.add(PoseFactor(PoseKey(j - 1, 0), PoseKey(j, 0), JointAngleKey(j, 0),
                         p_cost_model, jMi[j - 1], screw_axes[j - 1]));
  }
  // add tool pose factor (which is the pose goal)
  graph.add(
      ToolPoseFactor(PoseKey(dof, 0), tp_cost_model, jMi[dof], pose_goal));

  // set initial values for link COM poses
  poses[0] = Pose3(Rot3::Rz(M_PI / 2), Point3(0, 1, 0));
  poses[1] = Pose3(Rot3::Rz(M_PI / 2), Point3(0, 3, 0));

  // set initial values for joint angle q
  std::vector<double> joint_angles = {M_PI / 2, 0.0};

  Values init_values;
  init_values.insert(PoseKey(0, 0), base_pose);
  for (int j = 1; j <= dof; ++j) {
    init_values.insert(PoseKey(j, 0), poses[j - 1]);
    init_values.insert(JointAngleKey(j, 0), joint_angles[j - 1]);
  }

  EXPECT(assert_equal(
      Vector6::Zero(),
      boost::dynamic_pointer_cast<PoseFactor>(graph[2])->unwhitenedError(
          init_values),
      1e-6));

  EXPECT_DOUBLES_EQUAL(0, graph[2]->error(init_values), 1e-6);
  EXPECT_DOUBLES_EQUAL(0, graph.error(init_values), 1e-6);

  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();
  Vector2 actual_q;
  actual_q << result.atDouble(JointAngleKey(1, 0)),
      result.atDouble(JointAngleKey(2, 0));

  Vector2 expected_q;
  expected_q << M_PI / 2, 0.0;
  EXPECT(assert_equal(expected_q, actual_q, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}