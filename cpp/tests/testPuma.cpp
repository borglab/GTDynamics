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

// data
SignedDistanceField sdf;

/* ************************************************************************** */
TEST(ObstacleSDFFactorArm, data) {
  double cell_size = 0.02;
  // zero orgin
  Point3 origin(0, -0.7, -0.15);
  vector<Matrix> field(3);

  field[0] =
      (Matrix(7, 7) << 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
       0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236, 0.2000, 0.1000,
       -0.1000, -0.1000, -0.1000, 0.1000, 0.2000, 0.2000, 0.1000, -0.1000,
       -0.1000, -0.1000, 0.1000, 0.2000, 0.2000, 0.1000, -0.1000, -0.1000,
       -0.1000, 0.1000, 0.2000, 0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414,
       0.2236, 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828)
          .finished();
  field[1] =
      (Matrix(7, 7) << 0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000,
       0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449, 0.2236, 0.1414,
       0.1000, 0.1000, 0.1000, 0.1414, 0.2236, 0.2236, 0.1414, 0.1000, 0.1000,
       0.1000, 0.1414, 0.2236, 0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414,
       0.2236, 0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449, 0.3000,
       0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000)
          .finished();
  field[2] =
      (Matrix(7, 7) << 0.3464, 0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464,
       0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000, 0.2828, 0.2236,
       0.2000, 0.2000, 0.2000, 0.2236, 0.2828, 0.2828, 0.2236, 0.2000, 0.2000,
       0.2000, 0.2236, 0.2828, 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236,
       0.2828, 0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000, 0.3464,
       0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464)
          .finished();

  sdf = SignedDistanceField(origin, cell_size, field);
}

/**
 * Test motion planner for puma manipulator
 */
TEST(MotionPlanner, dh_puma) {
  // puma example
  vector<DH_Link> dh_puma = {
      DH_Link(0, 0.0000, 0.0000, +90, 'R', 0, Point3(0, 0, 0),
              Vector3(0, 0.35, 0).asDiagonal(), -180, 180, 2, 0.2, 0.02, 0.2,
              0.02, 100, 5),
      DH_Link(0, 0.4318, 0, 0.0, 'R', 17.40, Point3(-0.3638, 0.006, 0.2275),
              Vector3(0.13, 0.524, 0.539).asDiagonal(), -180, 180, 2, 0.2, 0.02,
              0.2, 0.02, 100, 5),
      DH_Link(0, 0.0203, 0.15005, -90, 'R', 4.80,
              Point3(-0.0203, -0.0141, 0.0700),
              Vector3(0.066, 0.086, 0.0125).asDiagonal(), -180, 180, 2, 0.2,
              0.02, 0.2, 0.02, 100, 5),
      DH_Link(0, 0, 0.4318, +90, 'R', 0.82, Point3(-0.19, 0, 0),
              Vector3(0.0018, 0.0013, 0.0018).asDiagonal(), -180, 180, 2, 0.2,
              0.02, 0.2, 0.02, 100, 5),
      DH_Link(0, 0.0000, 0.0000, -90, 'R', 0.34, Point3(0, 0, 0),
              Vector3(0.0003, 0.0004, 0.0003).asDiagonal(), -180, 180, 2, 0.2,
              0.02, 0.2, 0.02, 100, 5),
      DH_Link(0, 0.0000, 0.0000, 0.0, 'R', 0.09, Point3(0, 0, 0.032),
              Vector3(0.00015, 0.00015, 0.00004).asDiagonal(), -180, 180, 2,
              0.2, 0.02, 0.2, 0.02, 100, 5)};
  auto robot = Arm<DH_Link>(dh_puma);
  auto poses = robot.comFrames();
  Vector3 gravity;
  gravity << 0, 0, -9.8;
  Vector6 expected_q;
  expected_q << -30, 10, -50, -75, 60, -40;
  expected_q *= M_PI / 180;
  auto expected_T = robot.forwardKinematics(expected_q).back();
  Pose3 pose_goal(expected_T);
  auto dof = robot.numLinks();

  // TODO:(Mandy) need to take the shape of robot arm
  //      into consideration instead of naively using length.
  // generate sphere robot arm model
  vector<double> lengths, radii;
  radii.assign(dof, 0.05);
  double l;
  for (int i = 0; i < dof; ++i) {
    l = robot.link(i).length();
    lengths.push_back(l);
    if (l == 0) {
      radii[i] = 0;
    }
  }
  vector<vector<Point3>> sphere_centers_all = sphereCenters(lengths, radii);

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  // set Qc_model for GP
  opt.setQcModel(I_1x1);
  opt.setJointLimitCostModel(0.01);
  opt.setToolPoseCostModel(0.001);
  opt.setObstacleCostModel(0.001);
  opt.setCollisionEpsilon(0.2);

  MotionPlanner mp(opt);
  auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
                                            boost::none, gravity, sdf, sphere_centers_all, radii);
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
  saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, sdf);
  /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
#endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}