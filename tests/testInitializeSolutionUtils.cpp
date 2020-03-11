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

#include <string>
#include <iostream>

#include "gtdynamics/utils/initialize_solution_utils.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/dynamics/DynamicsGraph.h"


TEST(initialize_solution_utils, initialize_solution_interpolation) {
    using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
      simple_urdf_eq_mass::planar_axis;
    
    gtsam::Pose3 wTb_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
    gtsam::Pose3 wTb_f = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));

    double T_i = 0, T_f = 10, dt = 1;

    gtsam::Values init_vals = gtdynamics::initialize_solution_interpolation(
        my_robot, "l1", wTb_i, wTb_f, T_i, T_f, dt);

    int n_steps_init = static_cast<int>(std::round(T_i / dt));
    int n_steps_final = static_cast<int>(std::round(T_f / dt));

    for (int t = n_steps_init; t < n_steps_final; t++) {
        std::cout << init_vals.at(gtdynamics::PoseKey(my_robot.getLinkByName("l1")->getID(), t)).cast<gtsam::Pose3>() << std::endl;
    }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}