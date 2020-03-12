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
#include "gtdynamics/utils/initialize_solution_utils.h"

using gtsam::assert_equal;

TEST(initialize_solution_utils, initialize_solution_interpolation) {
  using simple_urdf::my_robot;

  gtsam::Pose3 wTb_i =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3());
  gtsam::Pose3 wTb_f = gtsam::Pose3(
      gtsam::Rot3::RzRyRx(M_PI, M_PI / 4, M_PI / 2), gtsam::Point3(1, 1, 1));

  double T_i = 0, T_f = 10, dt = 1;

  gtsam::Values init_vals = gtdynamics::initialize_solution_interpolation(
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

TEST(initialize_solution_utils, initialize_solution_interpolation_multi_phase) {
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
      gtdynamics::initialize_solution_interpolation_multi_phase(
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

TEST(initialize_solution_utils, initialize_solution_inverse_kinematics) {
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

  gtsam::Values init_vals = gtdynamics::initialize_solution_inverse_kinematics(
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
  //   EXPECT(assert_equal(
  //       wTb_t[0],
  //       init_vals.at(gtdynamics::PoseKey(l2->getID(), std::roundl(ts[0] /
  //       dt))).cast<gtsam::Pose3>())
  //   );
  //   EXPECT(assert_equal(
  //     0.0,
  //     (init_vals.at(gtdynamics::PoseKey(l1->getID(), std::roundl(ts[0] /
  //     dt))).cast<gtsam::Pose3>() * oTc_l1).translation().z(), 1e-3
  //   ));
}

TEST(initialize_solution_utils, initialize_solution_zero_values) {
  auto my_robot =
      gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  gtdynamics::ContactPoints contact_points = {
      gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      gtdynamics::zero_values(my_robot, 0, contact_points);

  for (auto&& link : my_robot.links())
    EXPECT(assert_equal(link->wTcom(),
                        init_vals.at(gtdynamics::PoseKey(link->getID(), 0))
                            .cast<gtsam::Pose3>()));

  for (auto&& joint : my_robot.joints())
    EXPECT(assert_equal(
        0.0, init_vals.atDouble(gtdynamics::JointAngleKey(joint->getID(), 0))));
}

TEST(initialize_solution_utils, initialize_solution_zero_values_trajectory) {
  auto my_robot =
      gtdynamics::Robot(std::string(URDF_PATH) + "/test/simple_urdf.urdf");

  auto l1 = my_robot.getLinkByName("l1");
  auto l2 = my_robot.getLinkByName("l2");

  gtsam::Pose3 wTb_i = l2->wTcom();

  gtsam::Pose3 oTc_l1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, -1.0));
  gtdynamics::ContactPoints contact_points = {
      gtdynamics::ContactPoint{l1->name(), oTc_l1.translation(), 1, 0.0}};

  gtsam::Values init_vals =
      gtdynamics::zero_values_trajectory(my_robot, 100, -1, contact_points);

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
