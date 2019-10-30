/**
 * @file  testMotionPlanner.cpp
 * @brief test robot arm motion planner using nonlinear factor graph
 * @Author: Mandy Xie
 */

#include <DhLink.h>
#include <MotionPlanner.h>
#include <UrdfLink.h>

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
  double cell_size;
  Point3 origin;
  vector<Matrix> field = readFromTxt(
      "../../../matlab/dataset/kukaDeskDataset.txt", origin, cell_size);
  sdf = SignedDistanceField(origin, cell_size, field);
}

// Test motion planner for kuka manipulator
TEST(MotionPlanner, urdf_kuka) {
  // KUKA example
  vector<UrdfLink> kuka = {
      UrdfLink(Pose3(Rot3(), Point3(0, 0, 0.1575)), Vector3(0, 0, 1), 'R', 4,
                Pose3(Rot3(), Point3(0, -0.03, 0.12)),
                Vector3(0.1, 0.09, 0.02).asDiagonal(), true, 0, 0,  
                -2.96705972839 * 180 / M_PI, 2.96705972839 * 180 / M_PI, 10,
                0.5, 0.1),
      UrdfLink(
          Pose3(Rot3::RzRyRx(1.57079632679, 0, 3.14159265359),
                Point3(0, 0, 0.2025)),
          Vector3(0, 0, 1), 'R', 4, Pose3(Rot3(), Point3(0.0003, 0.059, 0.042)),
          Vector3(0.05, 0.018, 0.044).asDiagonal(), true, 0, 0,  -2.09439510239 * 180 / M_PI,
          2.09439510239 * 180 / M_PI, 10, 0.5, 0.1),
      UrdfLink(Pose3(Rot3::RzRyRx(1.57079632679, 0, 3.14159265359),
                      Point3(0, 0.2045, 0)),
                Vector3(0, 0, 1), 'R', 3, Pose3(Rot3(), Point3(0, 0.03, 0.13)),
                Vector3(0.08, 0.075, 0.01).asDiagonal(), true, 0, 0, 
                -2.96705972839 * 180 / M_PI, 2.96705972839 * 180 / M_PI, 10,
                0.5, 0.1),
      UrdfLink(
          Pose3(Rot3::RzRyRx(1.57079632679, 0, 0), Point3(0, 0, 0.2155)),
          Vector3(0, 0, 1), 'R', 2.7, Pose3(Rot3(), Point3(0, 0.067, 0.034)),
          Vector3(0.03, 0.01, 0.029).asDiagonal(), true, 0, 0,  -2.09439510239 * 180 / M_PI,
          2.09439510239 * 180 / M_PI, 10, 0.5, 0.1),
      UrdfLink(Pose3(Rot3::RzRyRx(-1.57079632679, 3.14159265359, 0),
                      Point3(0, 0.1845, 0)),
                Vector3(0, 0, 1), 'R', 1.7,
                Pose3(Rot3(), Point3(0.0001, 0.021, 0.076)),
                Vector3(0.02, 0.018, 0.005).asDiagonal(), true, 0, 0, 
                -2.09439510239 * 180 / M_PI, 2.09439510239 * 180 / M_PI, 10,
                0.5, 0.1),
      UrdfLink(Pose3(Rot3::RzRyRx(1.57079632679, 0, 0), Point3(0, 0, 0.2155)),
                Vector3(0, 0, 1), 'R', 1.8,
                Pose3(Rot3(), Point3(0, 0.0006, 0.0004)),
                Vector3(0.005, 0.0036, 0.0047).asDiagonal(), true, 0, 0, 
                -2.09439510239 * 180 / M_PI, 2.09439510239 * 180 / M_PI, 10,
                0.5, 0.1),
      UrdfLink(Pose3(Rot3::RzRyRx(-1.57079632679, 3.14159265359, 0),
                      Point3(0, 0.081, 0)),
                Vector3(0, 0, 1), 'R', 0.3, Pose3(Rot3(), Point3(0, 0, 0.02)),
                Vector3(0.001, 0.001, 0.001).asDiagonal(), true, 0, 0, 
                -3.05432619099 * 180 / M_PI, 3.05432619099 * 180 / M_PI, 10,
                0.5, 0.1)};
  Pose3 base = Pose3(Rot3(), Point3(-0.1, 0, 0.07)),
        tool = Pose3(Rot3(), Point3(0, 0, 0.04));
  auto robot = Arm<UrdfLink>(kuka, base, tool);
  auto poses = robot.comFrames();
  Vector3 gravity;
  gravity << 0, 0, -9.8;
  Vector7 expected_q;
  expected_q << 0, 90, 0, 0, 0, 0, 0;
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
  for (int i = 1; i < dof + 1; ++i) {
    if (i < dof) {
      l = robot.link(i).length();
    } else {
      l = robot.tool().translation().norm();
    }
    lengths.push_back(l);
    if (l == 0) {
      radii[i - 1] = 0;
    }
  }
  vector<vector<Point3>> sphere_centers_all = sphereCenters(lengths, radii);

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  // set Qc_model for GP
  opt.setQcModel(1000 * I_1x1);
  opt.setJointLimitCostModel(0.01);
  opt.setToolPoseCostModel(0.001);
  opt.setObstacleCostModel(0.1);
  opt.setCollisionEpsilon(0.05);

  MotionPlanner mp(opt);
  auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
                                            boost::none, gravity, sdf, sphere_centers_all, radii);
  auto init_values =
      mp.factorGraphInitialization(robot, pose_goal, expected_q, boost::none);
  auto results = mp.factorGraphOptimization(graph, init_values);
  auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
  auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
  EXPECT(assert_equal(expected_T, actual_T, 1e-3));

  #if (DEBUG == 1)
    results.print("", MultiRobotKeyFormatter);
    graph.printErrors(results, "NonlinearFactorGraph: ",
    MultiRobotKeyFormatter);

    /* +++++++++++++++ output for v-rep visualization ++++++++++++++++ */
    string dir = "../../../v-rep/test_data/joint_angles/";
    saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, sdf);
    /* +++++++++++++++ output for v-rep visualization ++++++++++++++++ */
  #endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}