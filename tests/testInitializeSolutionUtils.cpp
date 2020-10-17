/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testInitializeSolutionUtils.cpp
 * @brief Test solution initialization utilities.
 * @Author: Alejandro Escontrela and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>
#include <iostream>
#include <string>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics; 
using gtsam::assert_equal;

TEST(InitializeSolutionUtils, InitializeSolutionInterpolation) {
  using simple_urdf::my_robot;

  gtsam::Pose3 wTb_i =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3());
  gtsam::Pose3 wTb_f = gtsam::Pose3(
      gtsam::Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2), gtsam::Point3(1, 1, 1));

  double T_i = 0, T_f = 10, dt = 1;

  gtsam::Values init_vals = InitializeSolutionInterpolation(
      my_robot, "l1", wTb_i, wTb_f, T_i, T_f, dt);

  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  gtsam::Pose3 T = init_vals.at<gtsam::Pose3>(
      PoseKey(my_robot.getLinkByName("l1")->getID(), 0));
  EXPECT(assert_equal(wTb_i, T));

  gtsam::Pose3 T_1 = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), 5));
  gtsam::Pose3 T_2 = gtsam::Pose3(wTb_i.rotation().slerp(0.5, wTb_f.rotation()),
                   gtsam::Point3(0.5, 0.5, 0.5));
  EXPECT(assert_equal( T_1, T_2));

  T_1 = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), n_steps_final - 1));
  T_2 = gtsam::Pose3(wTb_i.rotation().slerp(0.9, wTb_f.rotation()),
                   gtsam::Point3(0.9, 0.9, 0.9));
  EXPECT(assert_equal( T_1, T_2 ));

  T = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), n_steps_final));
  EXPECT(assert_equal(wTb_f, T));
}

TEST(InitializeSolutionUtils, InitializeSolutionInterpolationMultiPhase) {
  using simple_urdf_eq_mass::my_robot;

  gtsam::Pose3 wTb_i =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3());
  std::vector<gtsam::Pose3> wTb_t = {
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2),
                   gtsam::Point3(2, 1, 1))};
  std::vector<double> ts = {5, 10};
  double dt = 1;

  gtsam::Values init_vals =
      InitializeSolutionInterpolationMultiPhase(
          my_robot, "l1", wTb_i, wTb_t, ts, dt);

  gtsam::Pose3 T = init_vals.at<gtsam::Pose3>(
        PoseKey(my_robot.getLinkByName("l1")->getID(), 0));
  EXPECT(assert_equal(wTb_i, T));

  T = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l2")->getID(), 0));
  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 2, 0, 0),
                   gtsam::Point3(0, -1, 1)), T, 1e-3));

  T = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), 5));
  EXPECT(assert_equal(wTb_t[0], T));

  T = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), 9));
  EXPECT(assert_equal(
      gtsam::Pose3(wTb_t[0].rotation().slerp(0.8, wTb_t[1].rotation()),
                   gtsam::Point3(1.8, 1, 1)), T));

  T = init_vals.at<gtsam::Pose3>(
          PoseKey(my_robot.getLinkByName("l1")->getID(), 10));
  EXPECT(assert_equal(wTb_t[1], T));
}

TEST(InitializeSolutionUtils, InitializeSolutionInverseKinematics) {
  auto my_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();
  std::vector<gtsam::Pose3> wTb_t = {
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(1, 0, 2.5))};

  std::vector<double> ts = {10};
  double dt = 1;

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  ContactPoints contact_points = {
      ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  /**
   * The aim of this test is to initialize a trajectory for the simple two-link
   * that translates link l2 1 meter in the x direction and down 0.5 meters in
   * the y direction all the while ensuring that the end of link l1 remains in
   * contact with the ground. When initialized in it's upright position, the
   * two link robot is in a singular state. This is because the gradients of
   * the ContactKinematicsPoseFactor with respect to the x and y are equally 0.
   * This prevents link 1 from rotating about the revolute joint as to remain
   * in contact with the ground. This problem is addressed by adding a small
   * amount of gaussian noise to the initial solution, which prevents it from
   * being a singular configuration.
   *
   *  1. Desired trajectory obtained by adding noise to the initial solution:
   *  z                   |                 ı
   *  |                   | l2              |  l2
   *   ¯¯ x               |                 |
   *                      |       =>         \
   *                      | l1                \  l1
   *                      |                    \
   * ¯                  ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   *
   *  2. Optimized trajectory without noise in the solution initialization:
   *                      |
   *                      | l2              |
   *                      |                 | l2
   *                      |       =>        |
   *                      | l1              |
   *                      |                 | l1 :(
   *                   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|¯¯¯¯¯¯¯¯¯¯¯¯¯
   */
  double gaussian_noise = 1e-8;
  gtsam::Values init_vals = InitializeSolutionInverseKinematics(
      my_robot, l2->name(), wTb_i, wTb_t, ts, dt, gaussian_noise,
      contact_points);

  EXPECT(assert_equal(
      wTb_i, init_vals.at<gtsam::Pose3>(PoseKey(l2->getID(), 0)), 1e-3));

  gtsam::Pose3 T = init_vals.at<gtsam::Pose3>(PoseKey(l1->getID(), 0))*oTc_l1;
  EXPECT(assert_equal(0.0, T.translation().z(), 1e-3));

  double jangle = init_vals.atDouble(JointAngleKey(
                              my_robot.getJointByName("j1")->getID(), 0));
  EXPECT(assert_equal(0.0, jangle, 1e-3));

  for (int t = 0; t <= std::roundl(ts[0] / dt); t++){
    T = init_vals.at<gtsam::Pose3>(PoseKey(l1->getID(), t)) * oTc_l1;
    EXPECT(assert_equal(0.0, T.translation().z(),1e-3));
  }

  T = init_vals.at<gtsam::Pose3>(PoseKey(l2->getID(), std::roundl(ts[0] / dt)));
  EXPECT(assert_equal(wTb_t[0], T, 1e-3));
  T = (init_vals.at<gtsam::Pose3>(PoseKey(l1->getID(), std::roundl(ts[0] / dt))) *oTc_l1);
  EXPECT(assert_equal(0.0, T.translation().z(), 1e-3));
}

TEST(InitializeSolutionUtils, initialize_solution_zero_values) {
  auto my_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  ContactPoints contact_points = {
      ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      ZeroValues(my_robot, 0, 0.0, contact_points);

  gtsam::Pose3 T;
  double jangle;
  for (auto&& link : my_robot.links()){
      T = init_vals.at<gtsam::Pose3>(PoseKey(link->getID(), 0));
      EXPECT(assert_equal(link->wTcom(), T));
  }
    
  for (auto&& joint : my_robot.joints()){
      jangle = init_vals.atDouble(JointAngleKey(joint->getID(), 0));
      EXPECT(assert_equal(0.0, jangle));
  }  
}

TEST(InitializeSolutionUtils, initialize_solution_zero_values_trajectory) {
  auto my_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  ContactPoints contact_points = {
      ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      ZeroValuesTrajectory(my_robot, 100, -1, 0.0, contact_points);

  gtsam::Pose3 T;
  double jangle;
  for (int t = 0; t <= 100; t++) {
    for (auto&& link : my_robot.links()){
        T = init_vals.at<gtsam::Pose3>(PoseKey(link->getID(), t));
        EXPECT(assert_equal(link->wTcom(), T));
    }
    for (auto&& joint : my_robot.joints()){
        jangle = init_vals.atDouble(JointAngleKey(joint->getID(), t));
        EXPECT(assert_equal(0.0, jangle));
    }    
  }
}

TEST(initialize_solution_utils,
     initialize_solution_muli_phase_inverse_kinematics_trajectory) {
  auto my_robot =
      CreateRobotFromFile(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));

  ContactPoint c =
      ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0};
  ContactPoints p0 = {c};
  ContactPoints p1 = {};
  ContactPoints p2 = {c};

  std::vector<ContactPoints> phase_contact_points = {p0, p1, p2};
  std::vector<Robot> robots(3, my_robot);

  // Number of descretized timesteps for each phase.
  int steps_per_phase = 100;
  std::vector<int> phase_steps(3, steps_per_phase);
  double gaussian_noise = 1e-8;

  auto graph_builder = DynamicsGraph();
  // Transition graphs.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  std::vector<gtsam::NonlinearFactorGraph> transition_graphs;
  transition_graphs = {
      graph_builder.dynamicsFactorGraph(robots[0], 1 * steps_per_phase, gravity,
                                        boost::none, p0),
      graph_builder.dynamicsFactorGraph(robots[1], 2 * steps_per_phase, gravity,
                                        boost::none, p0),
  };

  std::vector<gtsam::Pose3> wTb_t;
  std::vector<int> ts;

  wTb_t.push_back(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0.2)));
  ts.push_back(3 * steps_per_phase);

  // Initial values for transition graphs.
  std::vector<gtsam::Values> transition_graph_init;
  transition_graph_init.push_back(ZeroValues(
      robots[0], 1 * steps_per_phase, gaussian_noise, p0));
  transition_graph_init.push_back(ZeroValues(
      robots[1], 2 * steps_per_phase, gaussian_noise, p0));

  gtsam::Values init_vals = MultiPhaseInverseKinematicsTrajectory(
      robots, l2->name(), phase_steps, wTb_i, wTb_t, ts, transition_graph_init,
      1. / 240, gaussian_noise, phase_contact_points);

  gtsam::Pose3 T = init_vals.at(PoseKey(l2->getID(), 0)).cast<gtsam::Pose3>();
  EXPECT(assert_equal(wTb_i, T, 1e-3));

  for (size_t i = 0; i < wTb_t.size(); i++){
    T = init_vals.at(PoseKey(l2->getID(), ts[i])).cast<gtsam::Pose3>();
    EXPECT(assert_equal(wTb_t[i], T, 1e-3));
  }

  // Make sure contacts respected during portions of the trajectory with contact
  // points.
  for (int i = 0; i < 100; i++) {  // Phase 0.
    gtsam::Pose3 wTol1 =
        init_vals.at(PoseKey(l1->getID(), i)).cast<gtsam::Pose3>();
    gtsam::Pose3 wTc = wTol1 * oTc_l1;
    EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-3));
  }
  for (int i = 200; i < 299; i++) {  // Phase 2.
    gtsam::Pose3 wTol1 =
        init_vals.at(PoseKey(l1->getID(), i)).cast<gtsam::Pose3>();
    gtsam::Pose3 wTc = wTol1 * oTc_l1;
    EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-3));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
