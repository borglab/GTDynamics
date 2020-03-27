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
#include "gtdynamics/utils/InitializeSolutionUtils.h"

using gtsam::assert_equal;

TEST(InitializeSolutionUtils, InitializeSolutionInterpolation) {
  using simple_urdf::my_robot;

  gtsam::Pose3 wTb_i =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3());
  gtsam::Pose3 wTb_f = gtsam::Pose3(
      gtsam::Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2), gtsam::Point3(1, 1, 1));

  double T_i = 0, T_f = 10, dt = 1;

  gtsam::Values init_vals = gtdynamics::InitializeSolutionInterpolation(
      my_robot, "l1", wTb_i, wTb_f, T_i, T_f, dt);

  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  EXPECT(assert_equal(wTb_i, init_vals
                                 .at(gtdynamics::PoseKey(
                                     my_robot.getLinkByName("l1")->getID(), 0))
                                 .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      gtsam::Pose3(wTb_i.rotation().slerp(0.5, wTb_f.rotation()),
                   gtsam::Point3(0.5, 0.5, 0.5)),
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(), 5))
          .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      gtsam::Pose3(wTb_i.rotation().slerp(0.9, wTb_f.rotation()),
                   gtsam::Point3(0.9, 0.9, 0.9)),
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(),
                                  n_steps_final - 1))
          .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      wTb_f, init_vals
                 .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(),
                                         n_steps_final))
                 .cast<gtsam::Pose3>()));
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
      gtdynamics::InitializeSolutionInterpolationMultiPhase(
          my_robot, "l1", wTb_i, wTb_t, ts, dt);

  EXPECT(assert_equal(wTb_i, init_vals
                                 .at(gtdynamics::PoseKey(
                                     my_robot.getLinkByName("l1")->getID(), 0))
                                 .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 2, 0, 0),
                   gtsam::Point3(0, -1, 1)),
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l2")->getID(), 0))
          .cast<gtsam::Pose3>(),
      1e-3));

  EXPECT(assert_equal(
      wTb_t[0],
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(), 5))
          .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      gtsam::Pose3(wTb_t[0].rotation().slerp(0.8, wTb_t[1].rotation()),
                   gtsam::Point3(1.8, 1, 1)),
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(), 9))
          .cast<gtsam::Pose3>()));

  EXPECT(assert_equal(
      wTb_t[1],
      init_vals
          .at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(), 10))
          .cast<gtsam::Pose3>()));
}

TEST(InitializeSolutionUtils, InitializeSolutionInverseKinematics) {
  auto my_robot =
      gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();
  std::vector<gtsam::Pose3> wTb_t = {
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(1, 0, 2.5))};

  std::vector<double> ts = {10};
  double dt = 1;

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  gtdynamics::ContactPoints contact_points = {
      gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals = gtdynamics::InitializeSolutionInverseKinematics(
      my_robot, l2->name(), wTb_i, wTb_t, ts, dt, contact_points);

  EXPECT(assert_equal(
      wTb_i,
      init_vals.at(gtdynamics::PoseKey(l2->getID(), 0)).cast<gtsam::Pose3>(),
      1e-3));
  EXPECT(assert_equal(
      0.0,
      (init_vals.at(gtdynamics::PoseKey(l1->getID(), 0)).cast<gtsam::Pose3>() *
       oTc_l1)
          .translation()
          .z()))
  EXPECT(assert_equal(0.0, init_vals.atDouble(gtdynamics::JointAngleKey(
                               my_robot.getJointByName("j1")->getID(), 0))));

  // TODO(aescontrela): These unit tests should work once the issue with the
  // ContactKinematicsPoseFactor is fixed.

  for (int t = 0; t <= std::roundl(ts[0] / dt); t++) {
    EXPECT(assert_equal(0.0,
                        (init_vals.at(gtdynamics::PoseKey(l1->getID(), t))
                             .cast<gtsam::Pose3>() *
                         oTc_l1)
                            .translation()
                            .z(),
                        1e-3));
    // std::cout << init_vals.at(gtdynamics::PoseKey(l2->getID(), t))
    //                  .cast<gtsam::Pose3>()
    //           << std::endl;
  }

  EXPECT(assert_equal(
      wTb_t[0],
      init_vals.at(gtdynamics::PoseKey(l2->getID(), std::roundl(ts[0] / dt)))
          .cast<gtsam::Pose3>()));
  EXPECT(assert_equal(
      0.0,
      (init_vals.at(gtdynamics::PoseKey(l1->getID(), std::roundl(ts[0] / dt)))
           .cast<gtsam::Pose3>() *
       oTc_l1)
          .translation()
          .z(),
      1e-3));
}

TEST(InitializeSolutionUtils, initialize_solution_zero_values) {
  auto my_robot =
      gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  gtdynamics::ContactPoints contact_points = {
      gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      gtdynamics::ZeroValues(my_robot, 0, contact_points);

  for (auto&& link : my_robot.links())
    EXPECT(assert_equal(link->wTcom(),
                        init_vals.at(gtdynamics::PoseKey(link->getID(), 0))
                            .cast<gtsam::Pose3>()));

  for (auto&& joint : my_robot.joints())
    EXPECT(assert_equal(
        0.0, init_vals.atDouble(gtdynamics::JointAngleKey(joint->getID(), 0))));
}

TEST(InitializeSolutionUtils, initialize_solution_zero_values_trajectory) {
  auto my_robot =
      gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  /**
   *
   *    |                 ı
   *    | l2              |  l2
   *    |                 |
   *    |       =>         \
   *    | l1                \  l1
   *    |                    \
   * ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   */

  /**
   *
   *    |
   *    | l2              |
   *    |                 | l2
   *    |       =>        |
   *    | l1              |
   *    |                 | l1 :(
   * ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|¯¯¯¯¯¯¯¯¯¯¯¯¯
   */

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  gtdynamics::ContactPoints contact_points = {
      gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      gtdynamics::ZeroValuesTrajectory(my_robot, 100, -1, contact_points);

  for (int t = 0; t <= 100; t++) {
    for (auto&& link : my_robot.links())
      EXPECT(assert_equal(link->wTcom(),
                          init_vals.at(gtdynamics::PoseKey(link->getID(), t))
                              .cast<gtsam::Pose3>()));

    for (auto&& joint : my_robot.joints())
      EXPECT(assert_equal(0.0, init_vals.atDouble(gtdynamics::JointAngleKey(
                                   joint->getID(), t))));
  }
}

TEST(initialize_solution_utils, initialize_solution_muli_phase_inverse_kinematics_trajectory) {
    auto my_robot = gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

    auto l1 = my_robot.getLinkByName("l1");
    auto l2 = my_robot.getLinkByName("l2");

    gtsam::Pose3 wTb_i = l2->wTcom();

    gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));

    gtdynamics::ContactPoint c = gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0};
    gtdynamics::ContactPoints p0 = {c};
    gtdynamics::ContactPoints p1 = { };
    gtdynamics::ContactPoints p2 = {c};

    std::vector<gtdynamics::ContactPoints> phase_contact_points = {p0, p1, p2};
    std::vector<gtdynamics::Robot> robots(3, my_robot);

    // Number of descretized timesteps for each phase.
    int steps_per_phase = 100;
    std::vector<int> phase_steps(3, steps_per_phase);

    auto graph_builder = gtdynamics::DynamicsGraph();
    // Transition graphs.
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    std::vector<gtsam::NonlinearFactorGraph> transition_graphs;
    transition_graphs = {
        graph_builder.dynamicsFactorGraph(robots[0], 1 * steps_per_phase, gravity, boost::none, p0),
        graph_builder.dynamicsFactorGraph(robots[1], 2 * steps_per_phase, gravity, boost::none, p0),
    };

    std::vector<gtsam::Pose3> wTb_t;
    std::vector<int> ts;

    wTb_t.push_back(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0.2)));
    ts.push_back(3 * steps_per_phase);

    // Initial values for transition graphs.
    std::vector<gtsam::Values> transition_graph_init;
    transition_graph_init.push_back(gtdynamics::ZeroValues(robots[0], 1 * steps_per_phase, p0));
    transition_graph_init.push_back(gtdynamics::ZeroValues(robots[1], 2 * steps_per_phase, p0));

    gtsam::Values init_vals = gtdynamics::MultiPhaseInverseKinematicsTrajectory(
        robots, l2->name(), phase_steps, wTb_i, wTb_t, ts,
        transition_graph_init, 1. / 240, phase_contact_points);

    EXPECT(assert_equal(
        wTb_i,
        init_vals.at(gtdynamics::PoseKey(l2->getID(), 0)).cast<gtsam::Pose3>()
    ));

    for (int i = 0; i < wTb_t.size(); i++)
        EXPECT(assert_equal(
            wTb_t[i],
            init_vals.at(gtdynamics::PoseKey(l2->getID(), ts[i])).cast<gtsam::Pose3>()
        ));

    // Make sure contacts respected during portions of the trajectory with contact points.
    for (int i = 0; i < 100; i++) {  // Phase 0.
        gtsam::Pose3 wTol1 = init_vals.at(gtdynamics::PoseKey(l1->getID(), i)).cast<gtsam::Pose3>();
        gtsam::Pose3 wTc = wTol1 * oTc_l1;
        EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-6));
    }
    for (int i = 200; i < 299; i++) {  // Phase 2.
        gtsam::Pose3 wTol1 = init_vals.at(gtdynamics::PoseKey(l1->getID(), i)).cast<gtsam::Pose3>();
        gtsam::Pose3 wTc = wTol1 * oTc_l1;
        EXPECT(assert_equal(0.0, wTc.translation().z(), 1e-6));
    }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
