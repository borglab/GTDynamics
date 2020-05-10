/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNonlinearDynamicsEliminateableGraph.cpp
 * @brief Test non-linear eliminateble dynamics factor graph
 * @Author: Mandy Xie
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/NonlinearDynamicsEliminateableGraph.h>
#include <gtdynamics/factors/TorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtdynamics/utils/utils.h>
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
using namespace std;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

namespace example {
// noise model
gtsam::noiseModel::Gaussian::shared_ptr cost_model =
    gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1);
}  // namespace example

// Test nonlinear eliminatebale dynamics graph
TEST(NonlinearDynamicsEliminateableGraph, constructor) {
  // // Create a torque factor
  // gtsam::Vector6 screw_axis;
  // screw_axis << 0, 0, 1, 0, 1, 0;
  // gtdynamics::TorqueFactor torque_factor(0, 1, example::cost_model,
  // screw_axis);
  // // Create a non-linear dynamic factor graph with only a torque factor
  // gtsam::NonlinearFactorGraph graph;
  // graph.push_back(torque_factor);
  // // Create a non-linear eliminatebale dynamic factor graph
  // auto NLEDG = NonlinearDynamicsEliminateableGraph(graph);
  auto NLEDG = NonlinearDynamicsEliminateableGraph();
  // peform elimination
  // auto chordal = NLEDG.eliminateSequential();
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
