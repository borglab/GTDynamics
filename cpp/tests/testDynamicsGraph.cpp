/**
 * @file  testDynamicsGraph.cpp
 * @brief test forward and inverse dynamics factor graph
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

using namespace std;
using namespace robot;
using namespace gtsam;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;


// Test forward dynamics with gravity
TEST(FD_factor_graph, simple_robot) {

  // Load the robot from urdf file
  UniversalRobot simple_robot = UniversalRobot("../../../urdfs/test/simple_urdf_eq_mass.urdf");

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
      wrenches = Vector6::Zero();
  Vector q = Vector::Zero(simple_robot.numJoints());
  Vector v = Vector::Zero(simple_robot.numJoints());
  Vector a = Vector::Zero(simple_robot.numJoints());
  Vector torque = Vector::Zero(simple_robot.numJoints());
  Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
  Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity, planar_axis);

  // specify known values
  for (auto link: simple_robot.links()) {
    int i = link -> getID();
    graph.add(PriorFactor<Pose3>(PoseKey(i, 0), link -> getComPose(), noiseModel::Constrained::All(6)));
    graph.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }
  for (auto joint: simple_robot.joints()) {
    int j = joint -> getID();
    graph.add(PriorFactor<double>(JointAngleKey(j, 0), 0, noiseModel::Constrained::All(1)));
    graph.add(PriorFactor<double>(JointVelKey(j, 0), 0, noiseModel::Constrained::All(1)));
    graph.add(PriorFactor<double>(TorqueKey(j, 0), 1, noiseModel::Constrained::All(1)));
  }

  // set initial values
  Values init_values = DynamicsGraphBuilder::zeroValues(simple_robot, 0);

  // using Guassian Newton optimizer
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  // print factor graph, values and error
  if (DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE) {
    for (auto& factor: graph) {
      for (auto& key: factor->keys()) {
        auto symb = LabeledSymbol(key);
        cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      }
      cout << "\n";
    }

    for (auto& key: result.keys()) {
      auto symb = LabeledSymbol(key);
      cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      result.at(key).print();
      cout << "\n";
    }
    cout << "error: " << graph.error(result) << "\n";
  }


  Vector actual_qAccel = Vector::Zero(simple_robot.numJoints());
  for (int j = 1; j <= simple_robot.numJoints(); ++j) {
    actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  }
  Vector expected_qAccel = (Vector(1)<<4).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));

  // check joint limit factors
  // Construct nonlinear factor graph with joint limit factors.
  gtsam::NonlinearFactorGraph joint_limit_factors = graph_builder.jointLimitFactors(simple_robot, 0);

  // 4 joint limit factors per joint (angle, velocity, acceleration, torque).
  EXPECT(assert_equal(
    (long) simple_robot.joints().size() * 4, 
    joint_limit_factors.keys().size()
  ));

}

TEST(FD_FACTOR_GRAPH, four_bar_linkage) {
  // Load the robot from urdf file
  UniversalRobot simple_robot = UniversalRobot("../../../urdfs/test/four_bar_linkage_pure.urdf");
  Vector torque = Vector::Zero(simple_robot.numJoints());
  Vector3 gravity = (Vector(3) << 0, 0, 0).finished();
  Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();

  // specify known values
  NonlinearFactorGraph prior_factors;
  for (auto link: simple_robot.links()) {
    int i = link -> getID();
    prior_factors.add(PriorFactor<Pose3>(PoseKey(i, 0), link -> getComPose(), noiseModel::Constrained::All(6)));
    prior_factors.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }
  for (auto joint: simple_robot.joints()) {
    int j = joint -> getID();
    prior_factors.add(PriorFactor<double>(JointAngleKey(j, 0), 0, noiseModel::Constrained::All(1)));
    prior_factors.add(PriorFactor<double>(JointVelKey(j, 0), 0, noiseModel::Constrained::All(1)));
    if ((j==1) || (j==3)) {
      prior_factors.add(PriorFactor<double>(TorqueKey(j, 0), 1, noiseModel::Constrained::All(1)));
    }
    else {
      prior_factors.add(PriorFactor<double>(TorqueKey(j, 0), 0, noiseModel::Constrained::All(1)));
    }
  }

  // set initial values
  Values init_values = DynamicsGraphBuilder::zeroValues(simple_robot, 0);
  
  // test the four bar linkage in the free-floating scenario
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  Vector actual_qAccel = Vector::Zero(simple_robot.numJoints());
  for (int j = 1; j <= simple_robot.numJoints(); ++j) {
    actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  }
  Vector expected_qAccel = (Vector(4)<<1, -1, 1, -1).finished();

  EXPECT(assert_equal(expected_qAccel, actual_qAccel));

  // A planar four bar linkage in 3D space should throw an ILS error with the
  // constraints specified.
  graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity);
  graph.add(prior_factors);
  GaussNewtonOptimizer optimizer1(graph, init_values);
  THROWS_EXCEPTION(optimizer1.optimize());

  // test the condition when we fix link "l1"
  simple_robot.getLinkByName("l1")->fix();
  graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);
  GaussNewtonOptimizer optimizer2(graph, init_values);
  optimizer2.optimize();
  result = optimizer2.values();
  actual_qAccel = Vector::Zero(simple_robot.numJoints());
  for (int j = 1; j <= simple_robot.numJoints(); ++j) {
    actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  }
  expected_qAccel = (Vector(4)<<0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(FD_FACTOR_GRAPH, jumping_robot) {
  // Load the robot from urdf file
  UniversalRobot jumping_robot = UniversalRobot("../../../urdfs/test/jumping_robot.urdf");
  jumping_robot.getLinkByName("l0")->fix();
  Vector torque = Vector::Zero(jumping_robot.numJoints());
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  Vector3 planar_axis = (Vector(3) << 1, 0, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();

  // specify known values
  NonlinearFactorGraph prior_factors;
  for (auto link: jumping_robot.links()) {
    int i = link -> getID();
    prior_factors.add(PriorFactor<Pose3>(PoseKey(i, 0), link -> getComPose(), noiseModel::Constrained::All(6)));
    prior_factors.add(PriorFactor<Vector6>(TwistKey(i, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  }

  double torque3 = 0;
  double torque2 = 0.5;
  Vector torques = (Vector(6)<<0, torque2, torque3, torque3, torque2, 0).finished();
  for (auto joint: jumping_robot.joints()) {
    int j = joint -> getID();
    prior_factors.add(PriorFactor<double>(JointAngleKey(j, 0), 0, noiseModel::Constrained::All(1)));
    prior_factors.add(PriorFactor<double>(JointVelKey(j, 0), 0, noiseModel::Constrained::All(1)));
    prior_factors.add(PriorFactor<double>(TorqueKey(j, 0), torques[j-1], noiseModel::Constrained::All(1)));
  }

  // set initial values
  Values init_values = DynamicsGraphBuilder::zeroValues(jumping_robot, 0);
  
  // test jumping robot FD
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(jumping_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

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