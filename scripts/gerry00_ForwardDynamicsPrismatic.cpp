/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  gerry00_ForwardDynamicsPrismatic.cpp
 * @brief Test forward dynamics optimization for an rpr robot
 * @Author: Frank Dellaert, Alejandro Escontrela, Gerry Chen
 */

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>

int main(int argc, char** argv) {

    // Load the robot and build a nonlinear factor graph of kinodynamics
    // constraints.
    auto simple_rpr = gtdynamics::Robot(
        "../../sdfs/test/simple_rpr.sdf", "simple_rpr_sdf");
    std::cout << "\033[1;31m" << "Robot Model:" << "\033[0m\n" << std::endl;
    simple_rpr.printRobot();

    auto graph_builder = gtdynamics::DynamicsGraph();
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
    auto kdfg = graph_builder.dynamicsFactorGraph(
        simple_rpr,
        0,  // timestep
        gravity);

    // Specify the forward dynamics priors and add them to the factor graph.
    gtsam::Vector theta = (gtsam::Vector(2) << 0, 1, 0).finished();
    gtsam::Vector theta_dot = (gtsam::Vector(2) << 0, 0, 0).finished();
    gtsam::Vector tau = (gtsam::Vector(2) << 0, 0, 0).finished();
    auto fd_priors = graph_builder.forwardDynamicsPriors(simple_rpr, 0,
        theta, theta_dot, tau);
    kdfg.add(fd_priors);

    // Initialize solution.
    auto init_values = graph_builder.zeroValues(simple_rpr, 0);

    // Compute the forward dynamics.
    gtsam::LevenbergMarquardtOptimizer optimizer(kdfg, init_values);
    gtsam::Values results = optimizer.optimize();
    // graph_builder.printValues(results);
    std::cout << "\033[1;31m" << "Joint Angles:" << "\033[0m" << std::endl;
    std::cout << graph_builder.jointAngles(simple_rpr, results, 0) << std::endl << std::endl;
    std::cout << "\033[1;31m" << "Link Poses:" << "\033[0m\n" << std::endl;
    results.at(gtdynamics::PoseKey(0, 0)).print();
    results.at(gtdynamics::PoseKey(1, 0)).print();
    results.at(gtdynamics::PoseKey(2, 0)).print();
    results.at(gtdynamics::PoseKey(3, 0)).print();

    return 0;
}