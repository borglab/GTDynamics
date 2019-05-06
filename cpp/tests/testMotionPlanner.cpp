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

// /**
//  * Test motion planner for DH_RR manipulator
//  */
// TEST(MotionPlanner, dh_rr) {
//   // RR link example
//   vector<DH_Link> dh_rr = {
//       DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180, 2, 0.15,
//               0.02, 0.12, 0.02, 50, 5),
//       DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), Z_3x3, -180, 180, 2, 0.15,
//               0.02, 0.12, 0.02, 50, 5)};
//   auto robot = Arm<DH_Link>(dh_rr);
//   Vector3 gravity;
//   gravity << 0, -9.8, 0;
//   Vector2 expected_q;
//   expected_q << 45, 45;
//   expected_q *= M_PI / 180;
//   auto expected_T = robot.forwardKinematics(expected_q).back();
//   Pose3 pose_goal(expected_T);
//   auto dof = robot.numLinks();

//   // motion planning optimization settings
//   OptimizerSetting opt = OptimizerSetting();
//   opt.setLM();
//   // set Qc_model for GP
//   opt.setQcModel(I_1x1);
//   opt.setJointLimitCostModel(0.01);
//   opt.setToolPoseCostModel(0.001);
//   opt.setObstacleCostModel(0.001);
//   opt.setSphereRadius(0.2);
//   opt.setCollisionEpsilon(0.2);

//   MotionPlanner mp(opt);
//   auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
//                                             boost::none, gravity, boost::none);
//   auto init_values = mp.factorGraphInitialization(
//       robot, pose_goal, Vector::Zero(dof), boost::none);
//   auto results = mp.factorGraphOptimization(graph, init_values);
//   auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
//   auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
//   EXPECT(assert_equal(expected_T, actual_T, 1e-3));

// #if (DEBUG == 1)
//   results.print("", MultiRobotKeyFormatter);
//   graph.printErrors(results, "NonlinearFactorGraph: ", MultiRobotKeyFormatter);

//   /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
//   string dir = "./";
//   saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, boost::none);
//   /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
// #endif
// }

// /**
//  * Test motion planner for puma manipulator
//  */
// TEST(MotionPlanner, dh_puma) {
//   // puma example
//   vector<DH_Link> dh_puma = {
//       DH_Link(0, 0.0000, 0.0000, +90, 'R', 0, Point3(0, 0, 0),
//               Vector3(0, 0.35, 0).asDiagonal(), -180, 180, 2, 0.2, 0.02, 0.2,
//               0.02, 100, 5),
//       DH_Link(0, 0.4318, 0, 0.0, 'R', 17.40, Point3(-0.3638, 0.006, 0.2275),
//               Vector3(0.13, 0.524, 0.539).asDiagonal(), -180, 180, 2, 0.2, 0.02,
//               0.2, 0.02, 100, 5),
//       DH_Link(0, 0.0203, 0.15005, -90, 'R', 4.80,
//               Point3(-0.0203, -0.0141, 0.0700),
//               Vector3(0.066, 0.086, 0.0125).asDiagonal(), -180, 180, 2, 0.2,
//               0.02, 0.2, 0.02, 100, 5),
//       DH_Link(0, 0, 0.4318, +90, 'R', 0.82, Point3(-0.19, 0, 0),
//               Vector3(0.0018, 0.0013, 0.0018).asDiagonal(), -180, 180, 2, 0.2,
//               0.02, 0.2, 0.02, 100, 5),
//       DH_Link(0, 0.0000, 0.0000, -90, 'R', 0.34, Point3(0, 0, 0),
//               Vector3(0.0003, 0.0004, 0.0003).asDiagonal(), -180, 180, 2, 0.2,
//               0.02, 0.2, 0.02, 100, 5),
//       DH_Link(0, 0.0000, 0.0000, 0.0, 'R', 0.09, Point3(0, 0, 0.032),
//               Vector3(0.00015, 0.00015, 0.00004).asDiagonal(), -180, 180, 2,
//               0.2, 0.02, 0.2, 0.02, 100, 5)};
//   auto robot = Arm<DH_Link>(dh_puma);
//   auto poses = robot.comFrames();
//   Vector3 gravity;
//   gravity << 0, 0, -9.8;
//   Vector6 expected_q;
//   expected_q << -30, 10, -50, -75, 60, -40;
//   expected_q *= M_PI / 180;
//   auto expected_T = robot.forwardKinematics(expected_q).back();
//   Pose3 pose_goal(expected_T);
//   auto dof = robot.numLinks();

//   // motion planning optimization settings
//   OptimizerSetting opt = OptimizerSetting();
//   opt.setLM();
//   // set Qc_model for GP
//   opt.setQcModel(I_1x1);
//   opt.setJointLimitCostModel(0.01);
//   opt.setToolPoseCostModel(0.001);
//   opt.setObstacleCostModel(0.001);
//   opt.setSphereRadius(0.05);
//   opt.setCollisionEpsilon(0.2);

//   MotionPlanner mp(opt);
//   auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
//                                             boost::none, gravity, sdf);
//   auto init_values = mp.factorGraphInitialization(
//       robot, pose_goal, Vector::Zero(dof), boost::none);
//   auto results = mp.factorGraphOptimization(graph, init_values);
//   auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
//   auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
//   EXPECT(assert_equal(expected_T, actual_T, 1e-3));

// #if (DEBUG == 1)
//   results.print("", MultiRobotKeyFormatter);
//   graph.printErrors(results, "NonlinearFactorGraph: ", MultiRobotKeyFormatter);

//   /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
//   string dir = "./";
//   saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, sdf);
//   /* +++++++++++++++ output for matlab visualization ++++++++++++++++ */
// #endif
// }

/**
 * Test motion planner for kuka manipulator
 */
TEST(MotionPlanner, urdf_kuka) {
  // KUKA example
  vector<URDF_Link> kuka = {
      URDF_Link(Pose3(Rot3(), Point3(0, 0, 0.1575)), Vector3(0, 0, 1), 'R', 4,
                Pose3(Rot3(), Point3(0, -0.03, 0.12)),
                Vector3(0.1, 0.09, 0.02).asDiagonal(),
                -2.96705972839 * 180 / M_PI, 2.96705972839 * 180 / M_PI, 10, 1,
                0.1),
      URDF_Link(
          Pose3(Rot3::RzRyRx(1.57079632679, 0, 3.14159265359),
                Point3(0, 0, 0.2025)),
          Vector3(0, 0, 1), 'R', 4, Pose3(Rot3(), Point3(0.0003, 0.059, 0.042)),
          Vector3(0.05, 0.018, 0.044).asDiagonal(), -2.09439510239 * 180 / M_PI,
          2.09439510239 * 180 / M_PI, 10, 1, 0.1),
      URDF_Link(Pose3(Rot3::RzRyRx(1.57079632679, 0, 3.14159265359),
                      Point3(0, 0.2045, 0)),
                Vector3(0, 0, 1), 'R', 3, Pose3(Rot3(), Point3(0, 0.03, 0.13)),
                Vector3(0.08, 0.075, 0.01).asDiagonal(),
                -2.96705972839 * 180 / M_PI, 2.96705972839 * 180 / M_PI, 10, 1,
                0.1),
      URDF_Link(
          Pose3(Rot3::RzRyRx(1.57079632679, 0, 0), Point3(0, 0, 0.2155)),
          Vector3(0, 0, 1), 'R', 2.7, Pose3(Rot3(), Point3(0, 0.067, 0.034)),
          Vector3(0.03, 0.01, 0.029).asDiagonal(), -2.09439510239 * 180 / M_PI,
          2.09439510239 * 180 / M_PI, 10, 1, 0.1),
      URDF_Link(Pose3(Rot3::RzRyRx(-1.57079632679, 3.14159265359, 0),
                      Point3(0, 0.1845, 0)),
                Vector3(0, 0, 1), 'R', 1.7,
                Pose3(Rot3(), Point3(0.0001, 0.021, 0.076)),
                Vector3(0.02, 0.018, 0.005).asDiagonal(),
                -2.09439510239 * 180 / M_PI, 2.09439510239 * 180 / M_PI, 10, 1,
                0.1),
      URDF_Link(
          Pose3(Rot3::RzRyRx(1.57079632679, 0, 0), Point3(0, 0, 0.2155)),
          Vector3(0, 0, 1), 'R', 1.8, Pose3(Rot3(), Point3(0, 0.0006, 0.0004)),
          Vector3(0.005, 0.0036, 0.0047).asDiagonal(),
          -2.09439510239 * 180 / M_PI, 2.09439510239 * 180 / M_PI, 10, 1, 0.1),
      URDF_Link(Pose3(Rot3::RzRyRx(-1.57079632679, 3.14159265359, 0),
                      Point3(0, 0.081, 0)),
                Vector3(0, 0, 1), 'R', 0.3, Pose3(Rot3(), Point3(0, 0, 0.02)),
                Vector3(0.001, 0.001, 0.001).asDiagonal(),
                -3.05432619099 * 180 / M_PI, 3.05432619099 * 180 / M_PI, 10, 1,
                0.1)};
  Pose3 base = Pose3(Rot3(), Point3(-0.1, 0, 0.07)), tool = Pose3(Rot3(), Point3(0, 0, 0.04));
  auto robot = Arm<URDF_Link>(kuka, base, tool);
  auto poses = robot.comFrames();
  Vector3 gravity;
  gravity << 0, 0, -9.8;
  Vector7 expected_q;
  expected_q << -120, 40, -60, -55, 150, -70, 90;
  expected_q *= M_PI / 180;
  auto expected_T = robot.forwardKinematics(expected_q).back();
  Pose3 pose_goal(expected_T);
  auto dof = robot.numLinks();

  // motion planning optimization settings
  OptimizerSetting opt = OptimizerSetting();
  opt.setLM();
  // set Qc_model for GP
  opt.setQcModel(1000*I_1x1);
  opt.setJointLimitCostModel(0.01);
  opt.setToolPoseCostModel(0.0001);
  opt.setObstacleCostModel(0.001);
  opt.setSphereRadius(0.05);
  opt.setCollisionEpsilon(0.2);

  MotionPlanner mp(opt);
  auto graph = mp.motionPlanningFactorGraph(robot, pose_goal, Vector::Zero(dof),
                                            boost::none, gravity);
  auto init_values =
      mp.factorGraphInitialization(robot, pose_goal, expected_q, boost::none);
  auto results = mp.factorGraphOptimization(graph, init_values);
  auto actual_q_trajectory = mp.extractTrajectoryQ(results, dof);
  auto actual_T = robot.forwardKinematics(actual_q_trajectory.back()).back();
  EXPECT(assert_equal(expected_T, actual_T, 1e-3));

#if (DEBUG == 1)
  results.print("", MultiRobotKeyFormatter);
  graph.printErrors(results, "NonlinearFactorGraph: ", MultiRobotKeyFormatter);

  /* +++++++++++++++ output for v-rep visualization ++++++++++++++++ */
  string dir =
      "../../../v-rep/test_data/joint_angles/";
  saveForVisualization(actual_q_trajectory, pose_goal, dof, dir, sdf);
  /* +++++++++++++++ output for v-rep visualization ++++++++++++++++ */
#endif
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}