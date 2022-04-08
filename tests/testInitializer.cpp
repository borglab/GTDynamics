/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testInitializeSolutionUtils.cpp
 * @brief Test solution initialization utilities.
 * @author Alejandro Escontrela and Yetong Zhang
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
#include "gtdynamics/utils/Initializer.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

double kNoiseSigma = 1e-8;

TEST(InitializeSolutionUtils, Interpolation) {
  Robot robot = simple_rr::getRobot();

  // Set initial and final values.
  Pose3 wTb_i;
  Rot3 wRb_f = Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2);
  Pose3 wTb_f(wRb_f, Point3(1, 1, 1));

  // We will interpolate from 0->10s, in 1 second increments.
  double T_i = 0, T_f = 10, dt = 1;

  Initializer initializer;
  gtsam::Values init_vals =
      initializer.InitializeSolutionInterpolation(robot, "link_0", wTb_i, wTb_f, T_i, T_f,
      dt);

  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  size_t id = 0;

  // Check start pose.
  EXPECT(assert_equal(wTb_i, Pose(init_vals, id)));

  // Check middle of trajectory.
  EXPECT(assert_equal(Pose3(wTb_i.rotation().slerp(0.5, wRb_f),
                            Point3(0.136439103437, 0.863560896563, 0.5)),
                      Pose(init_vals, id, 5)));

  // Check penultimate pose.
  EXPECT(
      assert_equal(Pose3(wTb_i.rotation().slerp(0.9, wRb_f),
                         Point3(0.794193007439, 1.03129011851,
                         0.961521708273)),
                   Pose(init_vals, id, n_steps_final - 1)));

  // Check end pose.
  EXPECT(assert_equal(wTb_f, Pose(init_vals, id, n_steps_final)));
}

TEST(InitializeSolutionUtils, InitializeSolutionInterpolationMultiPhase) {
  auto robot = simple_urdf_eq_mass::getRobot();
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 wTb_i;
  std::vector<Pose3> wTb_t = {
      Pose3(Rot3(), Point3(1, 1, 1)),
      Pose3(Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2), Point3(2, 1, 1))};
  std::vector<double> ts = {5, 10};
  double dt = 1;

  Initializer initializer;
  gtsam::Values init_vals = initializer.InitializeSolutionInterpolationMultiPhase(
      robot, "l1", wTb_i, wTb_t, ts, dt);

  Pose3 pose = Pose(init_vals, l1->id());
  EXPECT(assert_equal(wTb_i, pose));

  pose = Pose(init_vals, l2->id());
  EXPECT(assert_equal(Pose3(Rot3::RzRyRx(M_PI / 2, 0, 0), Point3(0, -1, 1)),
                      pose, 1e-3));

  pose = Pose(init_vals, l1->id(), 5);
  EXPECT(assert_equal(wTb_t[0], pose));

  pose = Pose(init_vals, l1->id(), 9);
  EXPECT(assert_equal(Pose3(wTb_t[0].rotation().slerp(0.8, wTb_t[1].rotation()),
                            Point3(1.83482681927, 1.03475261944, 1.1679796246)),
                      pose));

  pose = Pose(init_vals, l1->id(), 10);
  EXPECT(assert_equal(wTb_t[1], pose));
}

TEST(InitializeSolutionUtils, InitializePosesAndJoints) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));
  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 wTb_i = l2->bMcom();
  std::vector<Pose3> wTb_t = {Pose3(Rot3(), Point3(1, 0, 2.5)),
                              Pose3(Rot3(), Point3(2, 0, 2.5))};
  double t_i = 0.0;
  const std::vector<double> timesteps = {5, 10};
  const double dt = 1.0;
  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, kNoiseSigma);
  gtsam::Sampler sampler(sampler_noise_model);
  std::vector<Pose3> wTl_dt;

  Initializer initializer;
  auto actual = initializer.InitializePosesAndJoints(robot, wTb_i, wTb_t, l2->name(), t_i,
                                         timesteps, dt, sampler, &wTl_dt);
  gtsam::Values expected;
  InsertPose(&expected, 0, Pose3(Rot3(), Point3(0, 0, 1)));
  InsertPose(&expected, 1, Pose3(Rot3(), Point3(0, 0, 3)));
  InsertJointAngle(&expected, 0, 0.0);
  EXPECT(assert_equal(expected, actual, 1e-6));
  EXPECT_LONGS_EQUAL(18, wTl_dt.size());
}

TEST(InitializeSolutionUtils, InverseKinematics) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 wTb_i = l2->bMcom();
  std::vector<Pose3> wTb_t = {Pose3(Rot3(), Point3(1, 0, 2.5))};

  std::vector<double> ts = {10};
  double dt = 1;

  Pose3 oTc_l1(Rot3(), Point3(0, 0, -1.0));
  PointOnLinks contact_points = {{l1, oTc_l1.translation()}};

  /**
   * The aim of this test is to initialize a trajectory for the simple two-link
   * that translates link l2 1 meter in the x direction and down 0.5 meters in
   * the y direction all the while ensuring that the end of link l1 remains in
   * contact with the ground. When initialized in it's upright position, the
   * two link robot is in a singular state. This is because the gradients of
   * the ContactHeightFactor with respect to the x and y are equally 0.
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
  Initializer initializer;
  gtsam::Values init_vals = initializer.InitializeSolutionInverseKinematics(
      robot, l2->name(), wTb_i, wTb_t, ts, dt, kNoiseSigma, contact_points);

  EXPECT(assert_equal(wTb_i, Pose(init_vals, l2->id()), 1e-3));

  Pose3 pose = Pose(init_vals, l1->id()) * oTc_l1;
  EXPECT(assert_equal(0.0, pose.translation().z(), 1e-3));

  double joint_angle = JointAngle(init_vals, robot.joint("j1")->id());
  EXPECT(assert_equal(0.0, joint_angle, 1e-3));

  size_t T = std::roundl(ts[0] / dt);
  for (size_t t = 0; t <= T; t++) {
    pose = Pose(init_vals, l1->id(), t) * oTc_l1;
    EXPECT(assert_equal(0.0, pose.translation().z(), 1e-3));
  }

  pose = Pose(init_vals, l2->id(), T);
  EXPECT(assert_equal(wTb_t[0], pose, 1e-3));
  pose = Pose(init_vals, l1->id(), T) * oTc_l1;
  EXPECT(assert_equal(0.0, pose.translation().z(), 1e-3));
}

TEST(InitializeSolutionUtils, ZeroValues) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 wTb_i = l2->bMcom();

  Pose3 oTc_l1(Rot3(), Point3(0, 0, -1.0));
  PointOnLinks contact_points = {{l1, oTc_l1.translation()}};

  Initializer initializer;
  gtsam::Values init_vals = initializer.ZeroValues(robot, 0, 0.0, contact_points);

  Pose3 pose;
  double joint_angle;
  for (auto &&link : robot.links()) {
    pose = Pose(init_vals, link->id());
    EXPECT(assert_equal(link->bMcom(), pose));
  }

  for (auto &&joint : robot.joints()) {
    joint_angle = JointAngle(init_vals, joint->id());
    EXPECT(assert_equal(0.0, joint_angle));
  }
}

TEST(InitializeSolutionUtils, ZeroValuesTrajectory) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 wTb_i = l2->bMcom();

  Pose3 oTc_l1(Rot3(), Point3(0, 0, -1.0));
  PointOnLinks contact_points = {{l1, oTc_l1.translation()}};

  Initializer initializer;
  gtsam::Values init_vals =
      initializer.ZeroValuesTrajectory(robot, 100, -1, 0.0, contact_points);

  double joint_angle;
  for (size_t t = 0; t <= 100; t++) {
    for (auto &&link : robot.links()) {
      EXPECT(assert_equal(link->bMcom(), Pose(init_vals, link->id(), t)));
    }
    for (auto &&joint : robot.joints()) {
      joint_angle = JointAngle(init_vals, joint->id(), t);
      EXPECT(assert_equal(0.0, joint_angle));
    }
  }
}

TEST(InitializeSolutionUtils, MultiPhaseInverseKinematicsTrajectory) {
  auto robot =
      CreateRobotFromFile(kUrdfPath + std::string("test/simple_urdf.urdf"));

  auto l1 = robot.link("l1");
  auto l2 = robot.link("l2");

  Pose3 oTc_l1(Rot3(), Point3(0, 0, -1.0));

  Point3 c = oTc_l1.translation();
  PointOnLinks p0{{l1, c}};
  PointOnLinks p1{};
  PointOnLinks p2{{l1, c}};

  std::vector<PointOnLinks> phase_contact_points = {p0, p1, p2};

  // Number of descretized timesteps for each phase.
  int steps_per_phase = 100;
  std::vector<int> phase_steps(3, steps_per_phase);

  Pose3 wTb_i = l2->bMcom();

  std::vector<Pose3> wTb_t;
  std::vector<double> ts;

  wTb_t.push_back(Pose3(Rot3(), Point3(1, 0, 0.2)));

  ts.push_back(3 * steps_per_phase);

  // Initial values for transition graphs.
  gtdynamics::Initializer initializer;
  std::vector<gtsam::Values> transition_graph_init;
  transition_graph_init.push_back(
      initializer.ZeroValues(robot, 1 * steps_per_phase, kNoiseSigma, p0));
  transition_graph_init.push_back(
      initializer.ZeroValues(robot, 2 * steps_per_phase, kNoiseSigma, p0));

  double dt = 1.0;

  gtsam::Values init_vals = initializer.MultiPhaseInverseKinematicsTrajectory(
      robot, l2->name(), phase_steps, wTb_i, wTb_t, ts, transition_graph_init,
      dt, kNoiseSigma, phase_contact_points);

  Pose3 pose = Pose(init_vals, l2->id());
  EXPECT(assert_equal(wTb_i, pose, 1e-3));

  for (size_t t = 0; t < wTb_t.size(); t++) {
    pose = Pose(init_vals, l2->id(), ts[t]);
    EXPECT(assert_equal(wTb_t[t], pose, 1e-3));
  }

  // Make sure contacts respected during portions of the trajectory with contact
  // points.
  for (size_t t = 0; t < 100; t++) {  // Phase 0.
    Pose3 wTol1 = Pose(init_vals, l1->id(), t);
    Pose3 wTc = wTol1 * oTc_l1;
    EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-3));
  }
  for (size_t t = 200; t < 299; t++) {  // Phase 2.
    Pose3 wTol1 = Pose(init_vals, l1->id(), t);
    Pose3 wTc = wTol1 * oTc_l1;
    EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-3));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
