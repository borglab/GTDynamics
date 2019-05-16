/**
 * @file  testMotionPlanner.cpp
 * @brief test robot arm motion planner using nonlinear factor graph
 * @Author: Mandy Xie
 */

#include <DHLink.h>
#include <MotionPlanner.h>
#include <URDFLink.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

/**
 * Test motion planner for DH_RR manipulator
 */
TEST(MotionPlanner, dh_rr) {
  // RR link example
  vector<DH_Link> dh_rr = {
      DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180, 2, 0.15,
              0.02, 0.12, 0.02, 50, 5),
      DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180, 2, 0.15,
              0.02, 0.12, 0.02, 50, 5)};
  auto robot = Arm<DH_Link>(dh_rr);
  Vector3 gravity;
  gravity << 0, -9.8, 0;
  Vector2 expected_q;
  expected_q << 45, 45;
  expected_q *= M_PI / 180;
  auto expected_T = robot.forwardKinematics(expected_q).back();
  Pose3 pose_goal(expected_T);
  auto dof = robot.numLinks();

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  // set Qc_model for GP
  opt.setQcModel(I_1x1);
  opt.setJointLimitCostModel(0.01);
  opt.setToolPoseCostModel(0.001);
  opt.setObstacleCostModel(0.001);

  MotionPlanner mp(opt);
  auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
                                            boost::none, gravity);
  auto init_values = mp.factorGraphInitialization(
      robot, pose_goal, Vector::Zero(dof), boost::none);
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