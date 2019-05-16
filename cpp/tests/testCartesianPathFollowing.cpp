/**
 * @file  testMotionPlanner.cpp
 * @brief test robot arm motion planner using nonlinear factor graph
 * @Author: Mandy Xie
 */

#include <DHLink.h>
#include <MotionPlanner.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// RR link example
vector<DH_Link> dh_rrr = {
    DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180,
            2, 0.3, 0.02, 0.2, 0.02, 50, 5),
    DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180,
            2, 0.3, 0.02, 0.2, 0.02, 50, 5),
    DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180,
            2, 0.3, 0.02, 0.2, 0.02, 50, 5)};
}  // namespace example

/**
 * Test circle path for DH_RRR manipulator
 */
TEST(MotionPlanner, rrr_link_square) {
  auto robot = Arm<DH_Link>(example::dh_rrr);

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  opt.setQcModel(1000 * I_1x1);
  opt.setJointLimitCostModel(0.001);
  opt.setToolPoseCostModel(0.0001);
  opt.setObstacleCostModel(0.01);
  opt.setCollisionEpsilon(0.2);

  // generate cartesian path
  auto cartesianPath = square(opt.total_step, M_PI / 2, 4.0);
  auto expected_T = cartesianPath.back();
  Pose3 pose_goal(expected_T);

  MotionPlanner mp(opt);
  auto dof = robot.numLinks();
  Vector3 gravity;
  gravity << 0, -9.8, 0;
  // precomputed with inverse kinematics
  Vector q_init = (Vector(3) << -1.0472, 2.0944, -1.0472).finished();
  auto graph = mp.motionPlanningFactorGraph(
      robot, pose_goal, q_init, cartesianPath, gravity);
  auto init_values =
      mp.factorGraphInitialization(robot, pose_goal, q_init, cartesianPath);
  auto results = mp.factorGraphOptimization(graph, init_values);
  auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
  auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
  EXPECT(assert_equal(expected_T, actual_T, 1e-3));

#if (DEBUG == 1)
  results.print("", MultiRobotKeyFormatter);
  graph.printErrors(results, "NonlinearFactorGraph: ", MultiRobotKeyFormatter);

  /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
  string dir = "./";
  saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, boost::none);
  /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
#endif
}

/**
 * Test square path for DH_RRR manipulator
 */
TEST(MotionPlanner, rrr_link_circle) {
  auto robot = Arm<DH_Link>(example::dh_rrr);

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  opt.setQcModel(1000 * I_1x1);
  opt.setJointLimitCostModel(0.001);
  opt.setToolPoseCostModel(0.0001);
  opt.setObstacleCostModel(0.01);
  opt.setCollisionEpsilon(0.2);

  // generate cartesian path
  auto cartesianPath = circle(opt.total_step, M_PI / 2, 4.0);
  auto expected_T = cartesianPath.back();
  Pose3 pose_goal(expected_T);

  MotionPlanner mp(opt);
  auto dof = robot.numLinks();
  Vector3 gravity;
  gravity << 0, -9.8, 0;
  // precomputed with inverse kinematics
  Vector q_init = (Vector(3) << -1.0472, 2.0944, -1.0472).finished();
  auto graph = mp.motionPlanningFactorGraph(
      robot, pose_goal, q_init, cartesianPath, gravity);
  auto init_values =
      mp.factorGraphInitialization(robot, pose_goal, q_init, cartesianPath);
  auto results = mp.factorGraphOptimization(graph, init_values);
  auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
  auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
  EXPECT(assert_equal(expected_T, actual_T, 1e-3));

#if (DEBUG == 1)
  results.print("", MultiRobotKeyFormatter);
  graph.printErrors(results, "NonlinearFactorGraph: ", MultiRobotKeyFormatter);

  /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
  string dir = "./";
  saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, boost::none);
  /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
#endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}