/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNonlinearEliminateableDynamicsGraph.cpp
 * @brief Test non-linear eliminateble dynamics factor graph
 * @Author: Mandy Xie
 */

#include <gtdynamics/dynamics/NonlinearEliminateableDynamicsGraph.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/utils.h>
#include <gtdynamics/utils/initialize_solution_utils.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using namespace gtdynamics;
using namespace gtsam;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

// Test nonlinear eliminatebale dynamics graph
TEST(NonlinearEliminateableDynamicsGraph, constructor) {
  using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
      simple_urdf_eq_mass::planar_axis, simple_urdf_eq_mass::joint_angles,
      simple_urdf_eq_mass::joint_vels;
  gtsam::Vector torques = gtsam::Vector::Ones(my_robot.numJoints());
  // build the dynamics factor graph
  auto graph_builder = DynamicsGraph();
  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles,
                                                joint_vels, torques));
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links()) {
    int i = link->getID();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtdynamics::PoseKey(i, 0), link->wTcom(),
        graph_builder.opt().bp_cost_model));
    graph.add(gtsam::PriorFactor<gtsam::Vector6>(
        gtdynamics::TwistKey(i, 0), gtsam::Vector6::Zero(),
        graph_builder.opt().bv_cost_model));
  }
  auto NLEDG = NonlinearEliminateableDynamicsGraph(graph);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
