/**
 * @file  testJumpingRobot.cpp
 * @brief test forward dynamics for four-bar linkage
 * @Author: Yetong Zhang
 */

#include <DynamicsGraph.h>
#include <UniversalRobot.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <utils.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace robot;
using namespace gtsam;

// Test forward dynamics with gravity
TEST(FD_factor_graph, optimization) {

  // Load the robot from urdf file
  UniversalRobot jumping_robot = UniversalRobot("../../../urdfs/test/jumping_robot.urdf");
  jumping_robot.getLinkByName("l0")->fix();
  jumping_robot.printRobot();

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
      wrenches = Vector6::Zero();
  Vector q = Vector::Zero(jumping_robot.numJoints());
  Vector v = Vector::Zero(jumping_robot.numJoints());
  double torque3 = 0;
  double torque2 = 0.5;
  Vector torques = (Vector(6)<<0, torque2, torque3, torque3, torque2, 0).finished();
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(jumping_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(jumping_robot, 0, q, v, torques));
  DynamicsGraphBuilder::print_graph(graph);

  // set initial values
  Values init_values = DynamicsGraphBuilder::zeroValues(jumping_robot, 0);

  // optimize
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();
  cout << "error: " << graph.error(result) << "\n";

  // save graph for visualization
  DynamicsGraphBuilder::saveGraph("../../../visualization/factor_graph.json", graph, result, jumping_robot, 0, true);

  // check acceleration
  auto expected_qAccel = Vector(6);
  double m1 = 0.31;
  double m2 = 0.28;
  double m3 = 0.54;
  double link_radius = 0.02;
  double l = 0.55;
  double theta = 0.0 / 180.0 * M_PI;
  double acc =
      (torque3 - torque2 * 2 - (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * 9.8 * l * std::sin(theta)) /
      (std::pow(l, 2) * (1.0 / 4 * m1 + (1.0 / 4 + 2 * std::pow(std::sin(theta), 2)) * m2 + 2 * std::pow(std::sin(theta), 2) * m3) +
       (std::pow(l, 2) + 3 * std::pow(link_radius, 2)) * (1.0 / 12 * m1 + 1.0 / 12 * m2));
  expected_qAccel << acc, -2 * acc, acc, acc, -2 * acc, acc;

  Vector actual_qAccel = Vector::Zero(jumping_robot.numJoints());
  for (int j = 1; j <= jumping_robot.numJoints(); ++j) {
    actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  }
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}



int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}