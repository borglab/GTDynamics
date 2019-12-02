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
TEST(FD_factor_graph, optimization) {

  // Load the robot from urdf file
  UniversalRobot simple_robot = UniversalRobot("../../../urdfs/test/simple_urdf.urdf");

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
      wrenches = Vector6::Zero();
  Vector q = Vector::Zero(simple_robot.numJoints());
  Vector v = Vector::Zero(simple_robot.numJoints());
  Vector a = Vector::Zero(simple_robot.numJoints());
  Vector torque = Vector::Zero(simple_robot.numJoints());
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(simple_robot, 0, gravity);

  // specify known values
  graph.add(PriorFactor<Pose3>(PoseKey(1, 0), Pose3(), noiseModel::Constrained::All(6)));
  graph.add(PriorFactor<Vector6>(TwistKey(1, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  graph.add(PriorFactor<double>(JointAngleKey(3, 0), 0, noiseModel::Constrained::All(1)));
  graph.add(PriorFactor<double>(JointVelKey(3, 0), 0, noiseModel::Constrained::All(1)));
  graph.add(PriorFactor<double>(TorqueKey(3, 0), 0, noiseModel::Constrained::All(1)));

  // set initial values
  Values init_values;
  for (auto link: simple_robot.links()) {
    int i = link -> getID();

    init_values.insert(PoseKey(i, 0), Pose3());
    init_values.insert(TwistKey(i, 0), twists);
    init_values.insert(TwistAccelKey(i, 0), accels);
  }
  for (auto joint: simple_robot.joints()) {
    int j = joint -> getID();
    init_values.insert(WrenchKey(joint->parentLink()->getID(), j, 0), wrenches);
    init_values.insert(WrenchKey(joint->childLink().lock()->getID(), j, 0), wrenches);
    Vector torque0 = Vector::Zero(simple_robot.numJoints());
    torque0 << 1;
    init_values.insert(TorqueKey(j, 0), torque0[0]);
    init_values.insert(JointAngleKey(j, 0), q[0]);
    init_values.insert(JointVelKey(j, 0), v[0]);
    init_values.insert(JointAccelKey(j, 0), a[0]);
  }

  // graph.print("", MultiRobotKeyFormatter);
  if(DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE) {
    for (auto& factor: graph) {
      for (auto& key: factor->keys()) {
        auto symb = LabeledSymbol(key);
        cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      }
      cout << "\n";
    }
  }
  

  // using Guassian Newton optimizer
  GaussNewtonOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  Values result = optimizer.values();

  if (DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE) {
    for (auto& key: result.keys()) {
      auto symb = LabeledSymbol(key);
      cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      result.at(key).print();
      cout << "\n";
    }
    // result.print();

    cout << "error: " << graph.error(result) << "\n";
  }


  // Vector actual_qAccel = Vector::Zero(example::dof);
  // for (int j = 1; j <= example::dof; ++j) {
  //   actual_qAccel[j - 1] = result.atDouble(JointAccelKey(j, 0));
  // }
  // Vector expected_qAccel = example::known_qAccel;

  // EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(FD_FACTOR_GRAPH, four_bar_optimization) {
  UniversalRobot four_bar = UniversalRobot("../../../urdfs/test/four_bar_linkage.urdf");

  Vector twists = Vector6::Zero(), accels = Vector6::Zero(),
          wrenches = Vector6::Zero();
  Vector q = Vector::Zero(four_bar.numJoints());
  Vector v = Vector::Zero(four_bar.numJoints());
  Vector a = Vector::Zero(four_bar.numJoints());
  Vector torque = Vector::Zero(four_bar.numJoints());
  Vector3 gravity = (Vector(3) << 0, -9.8, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(four_bar, 0, gravity);

  // specify known values
  graph.add(PriorFactor<Pose3>(PoseKey(1, 0), Pose3(), noiseModel::Constrained::All(6)));
  graph.add(PriorFactor<Vector6>(TwistKey(1, 0), Vector6::Zero(), noiseModel::Constrained::All(6)));
  graph.add(PriorFactor<double>(JointAngleKey(6, 0), 0, noiseModel::Constrained::All(1)));
  graph.add(PriorFactor<double>(JointVelKey(6, 0), 0, noiseModel::Constrained::All(1)));
  graph.add(PriorFactor<double>(TorqueKey(6, 0), 0, noiseModel::Constrained::All(1)));

  // set initial values
  Values init_values;
  for (auto link: four_bar.links()) {
    int i = link -> getID();
    init_values.insert(PoseKey(i, 0), Pose3());
    init_values.insert(TwistKey(i, 0), twists);
    init_values.insert(TwistAccelKey(i, 0), accels);
  }
  for (auto joint: four_bar.joints()) {
    int j = joint -> getID();
    init_values.insert(WrenchKey(joint->parentLink()->getID(), j, 0), wrenches);
    init_values.insert(WrenchKey(joint->childLink().lock()->getID(), j, 0), wrenches);
    Vector torque0 = Vector::Zero(four_bar.numJoints());
    torque0 << 1;
    init_values.insert(TorqueKey(j, 0), torque0[0]);
    init_values.insert(JointAngleKey(j, 0), q[0]);
    init_values.insert(JointVelKey(j, 0), v[0]);
    init_values.insert(JointAccelKey(j, 0), a[0]);
  }

  if (DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE) {
    cout << "Four bar linkage factor graph:" << endl;
    for (auto& factor: graph) {
      for (auto& key: factor->keys()) {
        auto symb = LabeledSymbol(key);
        cout << symb.chr() << int(symb.label()) << "_" << symb.index() << " ";
      }
      cout << "\n";
    }
  }
  
  // A planar four bar linkage in 3D space should throw an ILS error with the
  // constraints specified.
  GaussNewtonOptimizer optimizer(graph, init_values);
  THROWS_EXCEPTION(optimizer.optimize());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}